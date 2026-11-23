#!/usr/bin/env python3
"""Bench test for M2a: compare rf2o laser odometry against PX4's GPS estimate.

Run during an M1 flight (GPS still on) with rf2o publishing to /odom_rf2o and
publish_tf false. Prints the position and heading difference once a second.
Both topics are ENU: /odom comes from the M1 odom_tf node.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCompare(Node):
    def __init__(self):
        super().__init__('odom_compare')
        self.declare_parameter('reference', '/odom')
        self.declare_parameter('test', '/odom_rf2o')
        self.ref = None
        self.tst = None
        self.worst = 0.0
        self.create_subscription(Odometry, self.get_parameter('reference').value,
                                 lambda m: setattr(self, 'ref', m), 10)
        self.create_subscription(Odometry, self.get_parameter('test').value,
                                 lambda m: setattr(self, 'tst', m), 10)
        self.create_timer(1.0, self.tick)

    def tick(self):
        if self.ref is None or self.tst is None:
            self.get_logger().info(f'waiting: reference={self.ref is not None} test={self.tst is not None}')
            return
        a, b = self.ref.pose.pose, self.tst.pose.pose
        dx = b.position.x - a.position.x
        dy = b.position.y - a.position.y
        err = math.hypot(dx, dy)
        dyaw = math.degrees(math.atan2(math.sin(yaw(b.orientation) - yaw(a.orientation)),
                                       math.cos(yaw(b.orientation) - yaw(a.orientation))))
        self.worst = max(self.worst, err)
        self.get_logger().info(
            f'PX4 ({a.position.x:+.2f}, {a.position.y:+.2f})  rf2o ({b.position.x:+.2f}, {b.position.y:+.2f})  '
            f'error {err:.2f} m (worst {self.worst:.2f})  yaw diff {dyaw:+.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = OdomCompare()
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
