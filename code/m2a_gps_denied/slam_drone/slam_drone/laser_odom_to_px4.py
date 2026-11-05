#!/usr/bin/env python3
"""Laser odometry -> PX4 external vision (Milestone 2a, GPS-denied).

Subscribes : /odom                            nav_msgs/Odometry from rf2o
                                              (frame odom = ENU, child base_link = FLU)
Publishes  : /fmu/in/vehicle_visual_odometry  px4_msgs/VehicleOdometry (NED / FRD)

Why the frames line up: rf2o's odom frame starts at the pose the drone has when
rf2o starts. The drone spawns at the Gazebo origin facing +x (east), so odom is
the Gazebo world ENU frame. Its NED version (x north, y east, z down) is PX4's
local frame, so the M1 corridor waypoints stay valid with GPS off.
=> Start rf2o with the drone sitting on the ground at the spawn point, before arming.

What we send to EKF2:
  * horizontal position (NED x, y). z is set to 0 with a huge variance because
    PX4 drops the whole position if any element is NaN; height comes from baro.
  * yaw (NED heading = 90 deg - ENU yaw). Roll/pitch are 0 (rf2o is 2D);
    EKF2 only uses the yaw part.
  * horizontal velocity from finite differences of the rf2o pose, in NED.
    rf2o's own twist is unusable on a drone: it only fills linear.x (forward
    speed), so sideways motion would read as zero. vz = 0 with a huge variance.

Timestamps: the XRCE agent converts timestamps from this computer's system
(wall) clock into PX4 time, so we stamp with the system clock and subtract the
scan's age (measured in sim time) for timestamp_sample. EKF2_EV_DELAY stays 0.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

NAN = float('nan')
BIG_VAR = 100.0  # variance for axes we don't measure (z position, z velocity)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    """ENU yaw (about +z up) from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LaserOdomToPx4(Node):
    def __init__(self):
        super().__init__('laser_odom_to_px4')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('px4_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('pos_std', 0.05)    # m      (EKF2 floor: EKF2_EVP_NOISE)
        self.declare_parameter('yaw_std', 0.05)    # rad    (EKF2 floor: EKF2_EVA_NOISE)
        self.declare_parameter('vel_std', 0.20)    # m/s    (EKF2 floor: EKF2_EVV_NOISE)
        self.declare_parameter('vel_alpha', 0.4)   # low-pass on velocity, 0..1 (1 = no smoothing)
        self.declare_parameter('max_age', 0.3)     # s, clamp on scan-to-now latency

        odom_topic = self.get_parameter('odom_topic').value
        px4_topic = self.get_parameter('px4_topic').value
        self.pos_var = self.get_parameter('pos_std').value ** 2
        self.yaw_var = self.get_parameter('yaw_std').value ** 2
        self.vel_var = self.get_parameter('vel_std').value ** 2
        self.alpha = self.get_parameter('vel_alpha').value
        self.max_age = self.get_parameter('max_age').value

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn('use_sim_time is false: scan latency correction will be wrong. '
                                   'Run with --ros-args -p use_sim_time:=true')

        # Same QoS as the px4_ros_com examples use for /fmu/in topics.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(VehicleOdometry, px4_topic, px4_qos)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.wall = Clock(clock_type=ClockType.SYSTEM_TIME)

        self.prev = None          # (t_sim, x_enu, y_enu) of the previous sample
        self.vel_enu = (0.0, 0.0)
        self.count = 0
        self.last = None          # last published (x_ned, y_ned, yaw_ned_deg)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'{odom_topic} (ENU) -> {px4_topic} (NED)')

    def on_odom(self, msg: Odometry):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        x_e = msg.pose.pose.position.x
        y_e = msg.pose.pose.position.y
        yaw_e = yaw_from_quat(msg.pose.pose.orientation)
        if not all(math.isfinite(v) for v in (x_e, y_e, yaw_e)):
            self.get_logger().error('rf2o sent NaN/inf pose: restart rf2o (see troubleshooting)',
                                    throttle_duration_sec=2.0)
            return

        # --- velocity by finite difference (ENU), low-pass filtered ---
        if self.prev is not None:
            dt = t - self.prev[0]
            if dt <= 1e-4:
                return  # duplicate sample
            if dt < 0.5:
                vx = (x_e - self.prev[1]) / dt
                vy = (y_e - self.prev[2]) / dt
                a = self.alpha
                self.vel_enu = (a * vx + (1 - a) * self.vel_enu[0],
                                a * vy + (1 - a) * self.vel_enu[1])
            else:
                self.vel_enu = (0.0, 0.0)  # gap in data: don't extrapolate
        self.prev = (t, x_e, y_e)

        # --- latency of this scan, measured in sim time ---
        age = (self.get_clock().now().nanoseconds * 1e-9) - t
        age = min(max(age, 0.0), self.max_age)

        now_us = self.wall.now().nanoseconds // 1000
        out = VehicleOdometry()
        out.timestamp = now_us
        out.timestamp_sample = now_us - int(age * 1e6)

        # ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        yaw_n = wrap_pi(math.pi / 2.0 - yaw_e)
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(y_e), float(x_e), 0.0]
        out.q = [math.cos(yaw_n / 2.0), 0.0, 0.0, math.sin(yaw_n / 2.0)]
        out.position_variance = [self.pos_var, self.pos_var, BIG_VAR]
        out.orientation_variance = [self.yaw_var, self.yaw_var, self.yaw_var]

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = [float(self.vel_enu[1]), float(self.vel_enu[0]), 0.0]
        out.velocity_variance = [self.vel_var, self.vel_var, BIG_VAR]
        out.angular_velocity = [NAN, NAN, NAN]

        out.reset_counter = 0
        out.quality = 100
        self.pub.publish(out)

        self.count += 1
        self.last = (y_e, x_e, math.degrees(yaw_n))

    def report(self):
        if self.last is None:
            self.get_logger().warn('No laser odometry yet. Is rf2o running and /scan arriving?')
            return
        n, e, h = self.last
        self.get_logger().info(f'{self.count / 5.0:.1f} Hz to PX4 | N {n:+.2f} m  E {e:+.2f} m  heading {h:+.1f} deg')
        self.count = 0


def main(args=None):
    rclpy.init(args=args)
    node = LaserOdomToPx4()
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
