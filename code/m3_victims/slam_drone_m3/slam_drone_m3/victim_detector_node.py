#!/usr/bin/env python3
"""M3 step 2: find person-sized LiDAR clusters in the 3D map and drop victim pins on the map.

Subscribes : /map_3d              (PointCloud2 from height_map_3d)
Publishes  : /victims/markers     (visualization_msgs/MarkerArray: red pins + labels, confirmed only)
             /victims/candidates  (MarkerArray: every low cluster with its size, for tuning)
             /victims/poses       (geometry_msgs/PoseArray of confirmed victims)
Writes     : ~/slam_drone/maps/victims.csv every time a new victim is confirmed
"""
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from slam_drone_m3.victim_core import DEFAULT_PARAMS, VictimTracker, detect


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('period', 1.0)
        self.declare_parameter('match_radius', 0.7)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('csv_path', os.path.expanduser('~/slam_drone/maps/victims.csv'))
        # optional ground truth for the report: flat list [x1, y1, x2, y2, ...] in map frame
        self.declare_parameter('ground_truth_xy', [0.0])
        for k, v in DEFAULT_PARAMS.items():
            self.declare_parameter(k, list(v) if isinstance(v, tuple) else v)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame = gp('map_frame')
        self.csv = gp('csv_path')
        gt = list(gp('ground_truth_xy'))
        self.truth = [(gt[i], gt[i + 1]) for i in range(0, len(gt) - 1, 2)] if len(gt) >= 2 else []
        self.P = {k: (tuple(gp(k)) if isinstance(v, tuple) else gp(k)) for k, v in DEFAULT_PARAMS.items()}
        self.tracker = VictimTracker(gp('match_radius'), int(gp('min_hits')))
        self.latest = None

        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, '/map_3d', self.on_map, latched)
        self.pub_pins = self.create_publisher(MarkerArray, '/victims/markers', latched)
        self.pub_cand = self.create_publisher(MarkerArray, '/victims/candidates', 10)
        self.pub_poses = self.create_publisher(PoseArray, '/victims/poses', latched)
        self.create_timer(gp('period'), self.step)
        self.get_logger().info('Victim detector running (waiting for /map_3d)')

    def on_map(self, msg):
        self.latest = msg

    def step(self):
        if self.latest is None or self.latest.width == 0:
            return
        pts = point_cloud2.read_points_numpy(self.latest, field_names=('x', 'y', 'z')).astype(np.float64)
        self.latest = None
        victims, cands = detect(pts, self.P)
        for t in self.tracker.update(victims):
            err = ''
            if self.truth:
                d = min(np.hypot(t['x'] - x, t['y'] - y) for x, y in self.truth)
                err = f', {d:.2f} m from nearest true victim'
            self.get_logger().warn(f"VICTIM {t['id']} FOUND ({t['label']}) at "
                                   f"x={t['x']:.2f} y={t['y']:.2f}{err}")
            self.write_csv()
        self.publish_pins()
        self.publish_candidates(cands)

    def write_csv(self):
        os.makedirs(os.path.dirname(self.csv), exist_ok=True)
        with open(self.csv, 'w') as f:
            f.write('id,label,x,y,error_to_truth_m\n')
            for t in self.tracker.confirmed():
                e = (min(np.hypot(t['x'] - x, t['y'] - y) for x, y in self.truth)
                     if self.truth else float('nan'))
                f.write(f"{t['id']},{t['label']},{t['x']:.3f},{t['y']:.3f},{e:.3f}\n")

    def _marker(self, ns, mid, mtype, x, y, z, sx, sy, sz, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def publish_pins(self):
        arr, poses = MarkerArray(), PoseArray()
        poses.header.frame_id = self.frame
        poses.header.stamp = self.get_clock().now().to_msg()
        red = (1.0, 0.1, 0.1, 1.0)
        for t in self.tracker.confirmed():
            i, x, y = t['id'], t['x'], t['y']
            arr.markers.append(self._marker('victim_zone', i, Marker.CYLINDER, x, y, 0.02,
                                            1.2, 1.2, 0.04, (1.0, 0.1, 0.1, 0.35)))
            arr.markers.append(self._marker('victim_pole', i, Marker.CYLINDER, x, y, 1.1,
                                            0.06, 0.06, 2.2, red))
            arr.markers.append(self._marker('victim_head', i, Marker.SPHERE, x, y, 2.3,
                                            0.35, 0.35, 0.35, red))
            txt = self._marker('victim_label', i, Marker.TEXT_VIEW_FACING, x, y, 2.75,
                               0, 0, 0.35, (1.0, 1.0, 1.0, 1.0))
            txt.text = f"VICTIM {i} ({t['label']})"
            arr.markers.append(txt)
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            p.orientation.w = 1.0
            poses.poses.append(p)
        self.pub_pins.publish(arr)
        self.pub_poses.publish(poses)

    def publish_candidates(self, cands):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for k, c in enumerate(cands):
            col = (0.1, 1.0, 0.1, 0.5) if c['label'] else (1.0, 0.85, 0.0, 0.35)
            b = self._marker('cand_box', k, Marker.CUBE, c['x'], c['y'], c['height'] / 2,
                             c['length'], c['width'], c['height'], col)
            yaw = c['yaw']
            b.pose.orientation.z, b.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            arr.markers.append(b)
            t = self._marker('cand_text', k, Marker.TEXT_VIEW_FACING, c['x'], c['y'],
                             c['height'] + 0.3, 0, 0, 0.18, (1.0, 1.0, 1.0, 1.0))
            t.text = (f"{c['label'] or 'reject'} h{c['height']:.2f} {c['length']:.2f}x{c['width']:.2f} "
                      f"top{c['top_fraction']:.2f}{' WALL' if c['touches_structure'] else ''}")
            arr.markers.append(t)
        self.pub_cand.publish(arr)


def main():
    rclpy.init()
    node = VictimDetector()
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
