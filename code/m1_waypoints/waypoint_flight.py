#!/usr/bin/env python3
"""
waypoint_flight.py  -  M1 step 5 (Kishan, PX4 side)

Flies the x500 around the corridor loop of building.sdf in PX4 OFFBOARD mode:
  stream setpoints at 10 Hz -> switch to OFFBOARD -> arm -> climb to 1.5 m
  -> fly the ENU waypoint loop at a slow, SLAM-friendly speed -> LAND -> exit.

Based on the px4_ros_com offboard example, with three additions:
  * a "carrot" that slides along each leg at cruise_speed, so PX4 never gets a
    6 m position jump (it would fly the leg at several m/s and smear the map);
  * it waits for the drone if the drone falls behind the carrot;
  * it stops and lets PX4 take over if the mode leaves OFFBOARD mid-flight.

Topics (PX4 v1.17, checked against src/modules/uxrce_dds_client/dds_topics.yaml):
  pub  /fmu/in/offboard_control_mode       px4_msgs/OffboardControlMode
  pub  /fmu/in/trajectory_setpoint         px4_msgs/TrajectorySetpoint
  pub  /fmu/in/vehicle_command             px4_msgs/VehicleCommand
  sub  /fmu/out/vehicle_status_v1          px4_msgs/VehicleStatus        (v1 suffix)
  sub  /fmu/out/vehicle_local_position_v1  px4_msgs/VehicleLocalPosition (v1 suffix)
All names are parameters, so a different suffix is a one-word change.

Frames: waypoints are given in Gazebo world ENU (x east, y north). The drone
spawns at the world origin and PX4's local origin is where it booted, so
local NED = (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

# Handoff Section 3.3 corridor loop, ENU metres, flown at `altitude`.
DEFAULT_WAYPOINTS_ENU = [0.0, 0.0,
                         6.2, 0.0,
                         6.2, 6.0,
                         -6.2, 6.0,
                         -6.2, 0.0,
                         0.0, 0.0]

NAN = float('nan')


def enu_to_ned(x_e, y_e, z_e):
    return [y_e, x_e, -z_e]


def dist(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


class WaypointFlight(Node):

    # Mission phases
    WAIT_PX4 = 'WAIT_PX4'        # waiting for position + status from PX4
    ENGAGE = 'ENGAGE'            # streaming setpoints, asking for OFFBOARD + ARM
    FLY = 'FLY'                  # following the path (takeoff leg + loop)
    HOLD = 'HOLD'                # hover at the last waypoint before landing
    LAND = 'LAND'                # PX4 AUTO_LAND, waiting for disarm
    DONE = 'DONE'

    def __init__(self):
        super().__init__('waypoint_flight')

        # ---------------- parameters ----------------
        p = self.declare_parameter
        p('altitude', 1.5)              # m above take-off point (walls are 3 m)
        p('cruise_speed', 0.5)          # m/s along each horizontal leg
        p('climb_speed', 0.5)           # m/s on the take-off leg
        p('acceptance_radius', 0.3)     # m, waypoint counts as reached
        p('max_lead', 1.0)              # m, carrot waits if drone is further behind
        p('hold_time', 3.0)             # s hover at the end before landing
        p('engage_delay', 1.0)          # s of setpoints before OFFBOARD request
        p('waypoints_enu', DEFAULT_WAYPOINTS_ENU)   # flat list x0,y0,x1,y1,...
        # PX4's local frame can be rotated a few degrees from the Gazebo world
        # (seen: ~6 deg). With align_to_world, the waypoints are rotated by
        # (PX4 yaw at engage - true world yaw at spawn) so the loop lines up
        # with the building. spawn_yaw_enu = drone's real yaw in the world
        # (0 = facing +x/east, which is how PX4 spawns the x500).
        p('align_to_world', True)
        p('spawn_yaw_enu', 0.0)         # rad
        p('topic_offboard_mode', '/fmu/in/offboard_control_mode')
        p('topic_setpoint', '/fmu/in/trajectory_setpoint')
        p('topic_command', '/fmu/in/vehicle_command')
        p('topic_status', '/fmu/out/vehicle_status_v1')
        p('topic_local_position', '/fmu/out/vehicle_local_position_v1')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.altitude = float(g('altitude'))
        self.cruise_speed = float(g('cruise_speed'))
        self.climb_speed = float(g('climb_speed'))
        self.accept_r = float(g('acceptance_radius'))
        self.max_lead = float(g('max_lead'))
        self.hold_time = float(g('hold_time'))
        self.engage_delay = float(g('engage_delay'))
        wp = list(g('waypoints_enu'))
        if len(wp) < 2 or len(wp) % 2:
            raise ValueError('waypoints_enu must be a flat list x0,y0,x1,y1,...')
        self.waypoints_enu = [(wp[i], wp[i + 1]) for i in range(0, len(wp), 2)]
        self.align = bool(g('align_to_world'))
        self.spawn_yaw = float(g('spawn_yaw_enu'))
        self.waypoints_ned = []     # filled in build_path() once heading is known

        # ---------------- PX4 QoS (as in px4_ros_com) ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_mode = self.create_publisher(
            OffboardControlMode, g('topic_offboard_mode'), qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, g('topic_setpoint'), qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, g('topic_command'), qos)
        self.create_subscription(
            VehicleStatus, g('topic_status'), self.on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, g('topic_local_position'), self.on_pos, qos)

        # ---------------- state ----------------
        self.status = None
        self.pos = None             # [x, y, z] NED
        self.heading = None         # rad, NED yaw
        self.phase = self.WAIT_PX4
        self.phase_t0 = self.now()
        self.last_req = -1e9
        self.ticks = 0

        self.hold_sp = None         # position to hold while engaging
        self.yaw = NAN
        self.path = []              # list of NED points, path[0] = start
        self.seg = 0                # current leg: path[seg] -> path[seg+1]
        self.carrot = None
        self.carrot_vel = [0.0, 0.0, 0.0]

        self.dt = 0.1               # 10 Hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'waypoint_flight ready: {len(self.waypoints_enu)} waypoints at '
            f'{self.altitude:.1f} m, {self.cruise_speed:.2f} m/s. '
            f'Listening on {g("topic_status")} and {g("topic_local_position")}')

    # ------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_phase(self, phase):
        self.get_logger().info(f'phase {self.phase} -> {phase}')
        self.phase = phase
        self.phase_t0 = self.now()

    def stamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def is_offboard(self):
        return (self.status is not None and
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def is_armed(self):
        return (self.status is not None and
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    # ------------------------------------------------------------ callbacks
    def on_status(self, msg):
        self.status = msg

    def on_pos(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.pos = [msg.x, msg.y, msg.z]
            self.heading = msg.heading

    # ------------------------------------------------------------ publishers
    def publish_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.stamp_us()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_mode.publish(m)

    def publish_setpoint(self, position, velocity=None):
        sp = TrajectorySetpoint()
        sp.timestamp = self.stamp_us()
        sp.position = [float(v) for v in position]
        sp.velocity = ([float(v) for v in velocity] if velocity is not None
                       else [NAN, NAN, NAN])
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = float(self.yaw)
        sp.yawspeed = NAN
        self.pub_sp.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0, p7=0.0):
        c = VehicleCommand()
        c.timestamp = self.stamp_us()
        c.command = command
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.param7 = float(p7)
        sysid = self.status.system_id if self.status and self.status.system_id else 1
        c.target_system = sysid
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def request_offboard(self):
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, PX4 custom main mode OFFBOARD = 6
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def request_arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def request_land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------ path follower
    def build_path(self):
        start = list(self.pos)
        takeoff = [start[0], start[1], -self.altitude]
        # yaw offset between PX4's local ENU frame and the Gazebo world
        rot = 0.0
        if self.align:
            px4_yaw_enu = math.pi / 2.0 - self.heading
            rot = math.atan2(math.sin(px4_yaw_enu - self.spawn_yaw),
                             math.cos(px4_yaw_enu - self.spawn_yaw))
        c, s_ = math.cos(rot), math.sin(rot)
        self.waypoints_ned = [enu_to_ned(c * x - s_ * y, s_ * x + c * y,
                                         self.altitude)
                              for x, y in self.waypoints_enu]
        self.get_logger().info(
            f'PX4 frame vs world yaw offset: {math.degrees(rot):+.1f} deg '
            f'({"waypoints rotated to match" if self.align else "NOT corrected"})')
        self.path = [start, takeoff] + self.waypoints_ned
        self.seg = 0
        self.carrot = list(start)
        self.get_logger().info('path (NED): ' + ' -> '.join(
            f'({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})' for p in self.path))

    def step_carrot(self):
        """Advance the carrot along the current leg. Returns True when the
        whole path is finished."""
        while True:
            if self.seg >= len(self.path) - 1:
                return True
            end = self.path[self.seg + 1]
            to_end = [e - c for e, c in zip(end, self.carrot)]
            remaining = math.sqrt(sum(v * v for v in to_end))
            speed = self.climb_speed if self.seg == 0 else self.cruise_speed

            if remaining < 1e-3:
                # carrot is at the leg's end: wait for the drone, then next leg
                self.carrot_vel = [0.0, 0.0, 0.0]
                if dist(self.pos, end) < self.accept_r:
                    name = 'take-off altitude' if self.seg == 0 else \
                        f'waypoint {self.seg}/{len(self.path) - 2}'
                    self.get_logger().info(
                        f'reached {name} ({end[0]:.1f},{end[1]:.1f},{end[2]:.1f})')
                    self.seg += 1
                    continue        # zero-length legs are skipped at once
                return False

            if dist(self.pos, self.carrot) > self.max_lead:
                # drone fell behind: hold the carrot until it catches up
                self.carrot_vel = [0.0, 0.0, 0.0]
                return False

            step = min(speed * self.dt, remaining)
            unit = [v / remaining for v in to_end]
            self.carrot = [c + u * step for c, u in zip(self.carrot, unit)]
            self.carrot_vel = [u * speed for u in unit]
            return False

    # ------------------------------------------------------------ main loop
    def tick(self):
        self.ticks += 1
        t = self.now() - self.phase_t0

        # PX4 needs the OffboardControlMode heartbeat (>2 Hz) the whole time
        # we want to be, or might become, in OFFBOARD.
        if self.phase not in (self.LAND, self.DONE):
            self.publish_mode()

        if self.phase == self.WAIT_PX4:
            if self.pos is None or self.status is None:
                if self.ticks % 30 == 0:
                    self.get_logger().warn(
                        'waiting for PX4: local position '
                        f'{"OK" if self.pos else "MISSING"}, status '
                        f'{"OK" if self.status else "MISSING"} '
                        '(is MicroXRCEAgent running? topic names right?)')
                return
            if self.is_armed():
                self.get_logger().error(
                    'vehicle is already armed/flying. Land it first '
                    '(commander land), then rerun this node.')
                self.set_phase(self.DONE)
                return
            self.hold_sp = list(self.pos)
            self.yaw = self.heading
            self.set_phase(self.ENGAGE)
            return

        if self.phase == self.ENGAGE:
            self.publish_setpoint(self.hold_sp)
            if t < self.engage_delay:
                return
            if self.is_offboard() and self.is_armed():
                self.get_logger().info('OFFBOARD + ARMED, taking off')
                self.build_path()
                self.set_phase(self.FLY)
                return
            if self.now() - self.last_req > 1.0:      # retry once a second
                self.last_req = self.now()
                if not self.is_offboard():
                    self.request_offboard()
                if not self.is_armed():
                    self.request_arm()
                self.get_logger().info(
                    f'requesting OFFBOARD + ARM (nav_state={self.status.nav_state}, '
                    f'arming_state={self.status.arming_state}, '
                    f'preflight_ok={self.status.pre_flight_checks_pass})')
            if t > 20.0:
                self.get_logger().error(
                    'could not get OFFBOARD + ARM in 20 s. Check the pxh> console '
                    'in Tab 1 for the reason. Giving up.')
                self.set_phase(self.DONE)
            return

        # From here on we are flying: never fight the pilot or a failsafe.
        if self.phase in (self.FLY, self.HOLD) and not self.is_offboard():
            self.get_logger().error(
                f'left OFFBOARD (nav_state={self.status.nav_state}); PX4 or the '
                'user took over. Stopping the mission.')
            self.set_phase(self.DONE)
            return

        if self.phase == self.FLY:
            finished = self.step_carrot()
            self.publish_setpoint(self.carrot, self.carrot_vel)
            if finished:
                self.set_phase(self.HOLD)
            return

        if self.phase == self.HOLD:
            self.publish_setpoint(self.path[-1])
            if t >= self.hold_time:
                self.get_logger().info('loop complete, landing')
                self.request_land()
                self.last_req = self.now()
                self.set_phase(self.LAND)
            return

        if self.phase == self.LAND:
            if not self.is_armed():
                self.get_logger().info('landed and disarmed. Mission complete.')
                self.set_phase(self.DONE)
                return
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND \
                    and self.now() - self.last_req > 1.0:
                self.last_req = self.now()
                self.request_land()
            return

        if self.phase == self.DONE:
            self.timer.cancel()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFlight()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
