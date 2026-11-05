#!/usr/bin/env python3
"""PX4 odometry -> ROS odometry + TF (Milestone 1).

Subscribes to PX4's VehicleOdometry (local NED frame, body FRD),
converts it to ROS conventions (odom = ENU, base_link = FLU) and publishes:
  * TF  odom -> base_link
  * nav_msgs/Odometry on /odom

In M2 this node is switched off and rf2o laser odometry provides odom -> base_link.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def qmul(a, b):
    """Hamilton product of quaternions given as (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    p = (0.0, v[0], v[1], v[2])
    r = qmul(qmul(q, p), qconj(q))
    return (r[1], r[2], r[3])


S = math.sqrt(0.5)
Q_NED_TO_ENU = (0.0, S, S, 0.0)   # swap x/y, flip z
Q_FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)  # 180 deg about x


def ned_to_enu(v):
    return (v[1], v[0], -v[2])


class OdomTf(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # PX4 publishes best-effort; VOLATILE subscriber is compatible with any durability.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(VehicleOdometry, topic, self.cb, px4_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.count = 0
        self.get_logger().info(f'Listening to {topic}, publishing {self.odom_frame} -> {self.base_frame} and /odom')

    def cb(self, msg: VehicleOdometry):
        # Ignore invalid samples (PX4 sends NaN before the EKF has a solution)
        if any(math.isnan(v) for v in list(msg.position) + list(msg.q)):
            return

        # Stamp with ROS time (= Gazebo sim time when use_sim_time is true),
        # so TF matches the /scan timestamps that SLAM uses.
        stamp = self.get_clock().now().to_msg()

        # Pose: NED/FRD -> ENU/FLU
        pos = ned_to_enu(msg.position)
        q_ned_frd = (float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3]))
        q = qmul(qmul(Q_NED_TO_ENU, q_ned_frd), Q_FRD_TO_FLU)
        n = math.sqrt(sum(c * c for c in q))
        q = tuple(c / n for c in q)

        # Linear velocity: PX4 gives it in NED (velocity_frame 1) or FRD body (3).
        # nav_msgs/Odometry wants it in the child frame (base_link, FLU).
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            vel_body = (msg.velocity[0], -msg.velocity[1], -msg.velocity[2])
        else:
            vel_body = rotate(qconj(q), ned_to_enu(msg.velocity))
        # Angular velocity: body FRD -> FLU
        w = msg.angular_velocity
        ang_body = (w[0], -w[1], -w[2])

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            self.tf_pub.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = float(pos[0])
        o.pose.pose.position.y = float(pos[1])
        o.pose.pose.position.z = float(pos[2])
        o.pose.pose.orientation.w = q[0]
        o.pose.pose.orientation.x = q[1]
        o.pose.pose.orientation.y = q[2]
        o.pose.pose.orientation.z = q[3]
        pv = [0.0 if math.isnan(v) else float(v) for v in msg.position_variance]
        ov = [0.0 if math.isnan(v) else float(v) for v in msg.orientation_variance]
        # NED variances -> ENU (swap x/y)
        o.pose.covariance[0] = pv[1]
        o.pose.covariance[7] = pv[0]
        o.pose.covariance[14] = pv[2]
        o.pose.covariance[21] = ov[0]
        o.pose.covariance[28] = ov[1]
        o.pose.covariance[35] = ov[2]
        o.twist.twist.linear.x = float(vel_body[0])
        o.twist.twist.linear.y = float(vel_body[1])
        o.twist.twist.linear.z = float(vel_body[2])
        o.twist.twist.angular.x = float(ang_body[0])
        o.twist.twist.angular.y = float(ang_body[1])
        o.twist.twist.angular.z = float(ang_body[2])
        self.odom_pub.publish(o)

        self.count += 1
        if self.count == 1:
            self.get_logger().info('First PX4 odometry received, TF is live')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTf()
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
