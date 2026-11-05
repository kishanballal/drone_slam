#!/usr/bin/env python3
"""cmd_vel -> PX4 offboard bridge (Milestone 2b).

The only node that talks to PX4 offboard in M2b. It:
  1. streams OffboardControlMode + TrajectorySetpoint at 20 Hz,
  2. switches PX4 to offboard, arms, and climbs to `altitude` (1.5 m),
  3. then flies whatever it is told, by priority:
       a) /cmd_vel (geometry_msgs/Twist, body FLU, from Nav2)
          -> NED velocity setpoint, altitude held by a z position setpoint
       b) /drone/position_target (geometry_msgs/PoseStamped, from the fallback
          frontier node) -> PX4 position setpoint
       c) nothing fresh -> hover in place
  4. lands on /drone/land (std_msgs/Empty).

Frames: Nav2 commands are in the body frame, so only the drone's heading is
needed to turn them into NED. Position targets are converted through the body
frame too (robot pose from TF, heading from PX4), so it does not matter whether
the SLAM map frame and PX4's local frame have the same origin or rotation.

If PX4 leaves offboard (you switch mode or land at the pxh> prompt) the bridge
stops commanding and never re-arms on its own. Restart it to take control again.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import Empty, String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
import tf2_ros
from tf2_ros import TransformException

from drone_explore.core import (wrap, clamp, clip_norm, yaw_from_quat, body_to_ned,
                                world_delta_to_body)

NAN = float('nan')


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        p = self.declare_parameter
        self.alt = float(p('altitude', 1.5).value)
        self.rate = float(p('rate_hz', 20.0).value)
        self.max_xy = float(p('max_xy_speed', 0.6).value)
        self.max_yaw = float(p('max_yaw_rate', 0.8).value)
        self.cmd_timeout = float(p('cmd_timeout', 0.5).value)
        self.target_timeout = float(p('target_timeout', 2.0).value)
        # Position mode: the setpoint is placed at most this far from the drone,
        # so it also caps speed (PX4 flies at roughly 0.95 x this, in m/s).
        self.max_step = float(p('max_target_step', 0.6).value)
        self.auto_takeoff = bool(p('auto_takeoff', True).value)
        self.stamped = bool(p('cmd_vel_stamped', False).value)
        self.base_frame = p('base_frame', 'base_link').value
        t_mode = p('topic_offboard_mode', '/fmu/in/offboard_control_mode').value
        t_sp = p('topic_setpoint', '/fmu/in/trajectory_setpoint').value
        t_cmd = p('topic_command', '/fmu/in/vehicle_command').value
        t_status = p('topic_status', '/fmu/out/vehicle_status_v1').value
        t_pos = p('topic_local_position', '/fmu/out/vehicle_local_position_v1').value

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, t_mode, px4_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, t_sp, px4_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, t_cmd, px4_qos)
        self.pub_state = self.create_publisher(String, '/drone/bridge_state', 10)

        self.create_subscription(VehicleStatus, t_status, self.on_status, px4_qos)
        self.create_subscription(VehicleLocalPosition, t_pos, self.on_pos, px4_qos)
        if self.stamped:
            self.create_subscription(TwistStamped, '/cmd_vel',
                                     lambda m: self.on_twist(m.twist), 10)
        else:
            self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.create_subscription(PoseStamped, '/drone/position_target', self.on_target, 10)
        self.create_subscription(Empty, '/drone/land', self.on_land, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status = None
        self.pos = None
        self.cmd = None
        self.cmd_time = None
        self.target = None
        self.target_time = None
        self.land_requested = False

        self.state = None
        self.set_state('WAIT_PX4')
        self.stream_ticks = 0
        self.hold_x = self.hold_y = 0.0
        self.yaw_sp = 0.0
        self.hovering = False
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'cmd_vel bridge up: altitude {self.alt} m, max {self.max_xy} m/s, '
            f'auto_takeoff={self.auto_takeoff}. Listening on {t_status}, {t_pos}')

    # ------------------------------------------------------------------ inputs
    def on_status(self, msg):
        self.status = msg

    def on_pos(self, msg):
        self.pos = msg

    def on_twist(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def on_target(self, msg):
        self.target = msg
        self.target_time = self.get_clock().now()

    def on_land(self, _msg):
        self.land_requested = True

    # ----------------------------------------------------------------- helpers
    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.get_logger().info(f'state -> {s}')
            self.pub_state.publish(String(data=s))

    def us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def age(self, t):
        if t is None:
            return float('inf')
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def send_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.us()
        m.position = True
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_mode.publish(m)

    def send_sp(self, position, velocity, yaw, yawspeed):
        sp = TrajectorySetpoint()
        sp.timestamp = self.us()
        sp.position = [float(v) for v in position]
        sp.velocity = [float(v) for v in velocity]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = float(yaw)
        sp.yawspeed = float(yawspeed)
        self.pub_sp.publish(sp)

    def hover(self):
        if not self.hovering:
            self.hold_x, self.hold_y = self.pos.x, self.pos.y
            self.yaw_sp = self.pos.heading
            self.hovering = True
        self.send_sp([self.hold_x, self.hold_y, -self.alt], [NAN, NAN, NAN], self.yaw_sp, 0.0)

    def send_command(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self.us()
        c.command = command
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def target_delta_ned(self):
        """(north, east) offset from the drone to the position target, or None."""
        frame = self.target.header.frame_id or 'map'
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {frame} -> {self.base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        robot_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        dx = self.target.pose.position.x - tr.x
        dy = self.target.pose.position.y - tr.y
        fwd, left = world_delta_to_body(dx, dy, robot_yaw)
        fwd, left = clip_norm(fwd, left, self.max_step)
        return body_to_ned(fwd, left, self.pos.heading)

    # -------------------------------------------------------------------- loop
    def tick(self):
        if self.pos is None or self.status is None:
            self.get_logger().info('Waiting for PX4 data (is MicroXRCEAgent running?)',
                                   throttle_duration_sec=3.0)
            return
        if self.state in ('LANDING', 'MANUAL'):
            return  # PX4 is in charge; stop streaming

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.land_requested:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.set_state('LANDING')
            return

        self.send_mode()

        if self.state == 'WAIT_PX4':
            self.hold_x, self.hold_y = self.pos.x, self.pos.y
            self.yaw_sp = self.pos.heading
            self.stream_ticks = 0
            self.set_state('ARMING')

        if self.state == 'ARMING':
            self.send_sp([self.hold_x, self.hold_y, -self.alt], [NAN] * 3, self.yaw_sp, 0.0)
            self.stream_ticks += 1
            if armed and offboard:
                self.set_state('TAKEOFF')
            elif not self.auto_takeoff:
                self.get_logger().info(
                    'Streaming. Type "commander mode offboard" then "commander arm" in pxh>',
                    throttle_duration_sec=5.0)
            elif self.stream_ticks % int(self.rate) == 0:   # every 1 s, after 1 s
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info('Requesting offboard + arm')
            return

        if not (armed and offboard):
            self.get_logger().warn('PX4 left offboard or disarmed. Bridge stopped '
                                   'commanding; restart it to take control again.')
            self.set_state('MANUAL')
            return

        if self.state == 'TAKEOFF':
            self.send_sp([self.hold_x, self.hold_y, -self.alt], [NAN] * 3, self.yaw_sp, 0.0)
            if abs(self.pos.z + self.alt) < 0.15:
                self.hovering = False
                self.set_state('READY')
                self.get_logger().info(
                    f'READY at {self.alt} m. Send /cmd_vel, or start exploration.')
            return

        # READY: fly the freshest command
        if self.age(self.cmd_time) < self.cmd_timeout:
            vx, vy = clip_norm(self.cmd[0], self.cmd[1], self.max_xy)
            yaw_rate = -clamp(self.cmd[2], self.max_yaw)      # FLU (CCW+) -> FRD (CW+)
            north, east = body_to_ned(vx, vy, self.pos.heading)
            if self.hovering or abs(wrap(self.yaw_sp - self.pos.heading)) > 0.6:
                self.yaw_sp = self.pos.heading
            self.yaw_sp = wrap(self.yaw_sp + yaw_rate * self.dt)
            self.hovering = False
            self.send_sp([NAN, NAN, -self.alt], [north, east, NAN], self.yaw_sp, yaw_rate)
        elif self.target is not None and self.age(self.target_time) < self.target_timeout:
            delta = self.target_delta_ned()
            if delta is None:
                self.hover()
                return
            if self.hovering:
                self.yaw_sp = self.pos.heading
            self.hovering = False
            self.send_sp([self.pos.x + delta[0], self.pos.y + delta[1], -self.alt],
                         [NAN] * 3, self.yaw_sp, 0.0)
        else:
            self.hover()


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
