#!/usr/bin/env python3
"""Dynamic obstacle tracker: finds things in the LiDAR scan that are not in the SLAM map.

  ros2 run drone_planning dynamic_tracker        (started by obstacles.launch.py)

At start it freezes a copy of /map (the building is explored by then). Every /scan
point that lands on free, mapped space more than wall_margin from a wall is a
candidate; candidates are clustered and tracked, giving each moving object an id
and a speed.

Publishes
  /dynamic_obstacles/markers  MarkerArray  orange cylinder + speed arrow + label per object
  /dynamic_obstacles          PoseArray    positions (map frame)
Logs
  DYNAMIC OBSTACLE #1 detected at (x, y)   and every few seconds its position + speed
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from drone_planning.tracker_core import StaticMap, Tracker, cluster, scan_points


class DynamicTracker(Node):

    def __init__(self):
        super().__init__('dynamic_tracker')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.wall_margin = float(p('wall_margin', 0.35).value)
        self.max_range = float(p('max_range', 8.0).value)
        self.refreeze = float(p('refreeze_period', 0.0).value)   # s, 0 = freeze once
        self.log_period = float(p('log_period', 3.0).value)
        self.tracker = Tracker(gate=float(p('gate', 1.0).value),
                               confirm=int(p('confirm_hits', 3).value),
                               timeout=float(p('timeout', 1.5).value))
        self.static = None
        self.frozen_at = None
        self.last_log = {}
        self.marker_ids = set()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value, self.on_map, latched)
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.pub_m = self.create_publisher(MarkerArray, '/dynamic_obstacles/markers', 10)
        self.pub_p = self.create_publisher(PoseArray, '/dynamic_obstacles', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.get_logger().info('dynamic_tracker up: waiting for /map and /scan')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg):
        t = self.now_s()
        if self.static is not None and (self.refreeze <= 0 or t - self.frozen_at < self.refreeze):
            return
        i = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width)
        self.static = StaticMap(grid, i.resolution, i.origin.position.x, i.origin.position.y,
                                wall_margin=self.wall_margin)
        if self.frozen_at is None:
            self.get_logger().info(f'map frozen ({i.width} x {i.height}); '
                                   'anything not in it is reported as a dynamic obstacle')
        self.frozen_at = t

    def on_scan(self, msg):
        if self.static is None:
            return
        try:
            tf = self.tf.lookup_transform(self.map_frame, msg.header.frame_id, Time())
        except TransformException:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        pts = [pt for pt in scan_points(msg.ranges, msg.angle_min, msg.angle_increment, pose,
                                        rmin=max(0.2, msg.range_min),
                                        rmax=min(self.max_range, msg.range_max))
               if self.static.is_dynamic_candidate(*pt)]
        t = self.now_s()
        new, dropped = self.tracker.update(cluster(pts), t)
        tracks = self.tracker.confirmed()
        for tid in new:
            tr = tracks[tid]
            self.get_logger().warn(f'DYNAMIC OBSTACLE #{tid} detected at ({tr["x"]:.1f}, {tr["y"]:.1f})')
            self.last_log[tid] = t
        for tid, tr in tracks.items():
            if t - self.last_log.get(tid, 0.0) > self.log_period:
                self.last_log[tid] = t
                self.get_logger().info(
                    f'DYNAMIC OBSTACLE #{tid} at ({tr["x"]:.1f}, {tr["y"]:.1f}) '
                    f'moving {Tracker.speed(tr):.2f} m/s')
        for tid in dropped:
            self.get_logger().info(f'DYNAMIC OBSTACLE #{tid} out of sight')
            self.last_log.pop(tid, None)
        self.publish(tracks, msg.header.stamp)

    def publish(self, tracks, stamp):
        ma = MarkerArray()
        live = set()
        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = self.map_frame, stamp
        for tid, tr in tracks.items():
            x, y, v = tr['x'], tr['y'], Tracker.speed(tr)
            ps = Pose()
            ps.position.x, ps.position.y = x, y
            ps.orientation.w = 1.0
            pa.poses.append(ps)
            for k, (typ, mid) in enumerate(((Marker.CYLINDER, tid * 3),
                                            (Marker.ARROW, tid * 3 + 1),
                                            (Marker.TEXT_VIEW_FACING, tid * 3 + 2))):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.map_frame, stamp
                m.ns, m.id, m.type, m.action = 'dynamic_obstacles', mid, typ, Marker.ADD
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.85
                if typ == Marker.CYLINDER:
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 1.0
                    m.scale.x, m.scale.y, m.scale.z = 0.6, 0.6, 2.0
                elif typ == Marker.ARROW:
                    m.points = [Point(x=x, y=y, z=2.1),
                                Point(x=x + 2.0 * tr['vx'], y=y + 2.0 * tr['vy'], z=2.1)]
                    m.scale.x, m.scale.y, m.scale.z = 0.08, 0.18, 0.2
                    m.color.r, m.color.g, m.color.b = 1.0, 0.2, 0.0
                else:
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 2.5
                    m.scale.z = 0.35
                    m.color.r = m.color.g = m.color.b = 1.0
                    m.text = f'#{tid}  {v:.1f} m/s'
                ma.markers.append(m)
                live.add(mid)
        for mid in self.marker_ids - live:
            m = Marker()
            m.header.frame_id, m.ns, m.id, m.action = self.map_frame, 'dynamic_obstacles', mid, Marker.DELETE
            ma.markers.append(m)
        self.marker_ids = live
        self.pub_m.publish(ma)
        self.pub_p.publish(pa)


def main():
    rclpy.init()
    node = DynamicTracker()
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
