#!/usr/bin/env python3
"""M3 step 1: accumulate /lidar_3d/points in the map frame into a height-coloured 3D voxel map.

Subscribes : /lidar_3d/points (sensor_msgs/PointCloud2, any frame that TF can link to 'map')
             /fmu/out/vehicle_odometry (PX4: height + roll/pitch, because with GPS off the
             rf2o TF is flat: z = 0 and no tilt. Set level_with_px4:=false to use TF only.)
Publishes  : /map_3d          (sensor_msgs/PointCloud2, map frame, fields x y z rgb, 1 Hz, latched)
Services   : /map_3d/save  (std_srvs/Trigger) -> writes ~/slam_drone/maps/map3d_<time>.ply
             /map_3d/clear (std_srvs/Trigger) -> empties the map (e.g. after a bad loop closure)
"""
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from slam_drone_m3.map3d_core import (VoxelMap, filter_sensor_points, floor_offset, height_colors,
                                      level_rotation, pack_xyzrgb, quat_to_rot,
                                      roll_pitch_from_px4, transform_points, write_ply,
                                      yaw_from_quat, yaw_rotation)

try:
    from px4_msgs.msg import VehicleOdometry
except ImportError:  # px4_msgs not sourced: TF-only mode
    VehicleOdometry = None


class HeightMapNode(Node):
    def __init__(self):
        super().__init__('height_map_3d')
        self.declare_parameter('cloud_topic', '/lidar_3d/points')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('min_range', 0.6)       # drops hits on the drone's own arms/props
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('z_min', -0.2)
        self.declare_parameter('z_max', 3.2)
        self.declare_parameter('color_z_min', 0.0)
        self.declare_parameter('color_z_max', 3.0)
        self.declare_parameter('min_hits', 2)          # voxel must be seen in >= 2 scans
        self.declare_parameter('process_every_n', 2)   # 10 Hz lidar -> 5 Hz insert
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('save_dir', os.path.expanduser('~/slam_drone/maps'))
        self.declare_parameter('level_with_px4', True)   # height + roll/pitch from PX4 (GPS-off fix)
        self.declare_parameter('px4_odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('floor_snap', True)       # shift each scan so the floor is at z = 0
        self.declare_parameter('publish_min_z', 0.1)     # /map_3d leaves out the floor (saves CPU + RViz)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = gp('map_frame')
        self.min_range, self.max_range = gp('min_range'), gp('max_range')
        self.cz = (gp('color_z_min'), gp('color_z_max'))
        self.min_hits = int(gp('min_hits'))
        self.every_n = max(1, int(gp('process_every_n')))
        self.save_dir = gp('save_dir')
        self.map = VoxelMap(gp('voxel_size'), gp('z_min'), gp('z_max'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(PointCloud2, gp('cloud_topic'), self.on_cloud, sensor_qos)
        self.base_frame = gp('base_frame')
        self.pub_min_z = float(gp('publish_min_z'))
        self.floor_snap = bool(gp('floor_snap'))
        self.level = bool(gp('level_with_px4')) and VehicleOdometry is not None
        if gp('level_with_px4') and VehicleOdometry is None:
            self.get_logger().error('px4_msgs not found: 3D map uses TF only (wrong height with GPS off)')
        self.px4 = None          # (altitude m, roll, pitch)
        self.px4_hist = deque(maxlen=60)   # (ros time s, alt, roll, pitch) to match scan stamps
        self.floor_corr = 0.0    # smoothed floor correction (m)
        self.floor_seen = 0
        if self.level:
            self.create_subscription(VehicleOdometry, gp('px4_odom_topic'), self.on_px4, sensor_qos)
        self.pub = self.create_publisher(PointCloud2, '/map_3d', latched)
        self.create_service(Trigger, '/map_3d/save', self.on_save)
        self.create_service(Trigger, '/map_3d/clear', self.on_clear)
        self.create_timer(gp('publish_period'), self.publish)

        self.count = 0
        self.used = 0
        self.tf_fail = 0
        self.get_logger().info(f"3D map: listening on {gp('cloud_topic')}, map frame '{self.map_frame}'")

    def on_px4(self, msg):
        roll, pitch = roll_pitch_from_px4(*[float(v) for v in msg.q])
        z = float(msg.position[2])
        if np.isfinite(z) and np.isfinite(roll) and np.isfinite(pitch):
            self.px4 = (-z, roll, pitch)   # NED z down -> altitude up
            self.px4_hist.append((self.get_clock().now().nanoseconds * 1e-9, -z, roll, pitch))

    def on_cloud(self, msg):
        self.count += 1
        if self.count % self.every_n:
            return
        if self.level:
            self.on_cloud_level(msg)
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=0.1))
        except Exception as e:  # noqa: BLE001  (tf2 raises several types)
            self.tf_fail += 1
            self.get_logger().warn(f'TF {self.map_frame} <- {msg.header.frame_id} not available: {e}',
                                   throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = filter_sensor_points(pts.astype(np.float64), self.min_range, self.max_range)
        if len(pts) == 0:
            return
        q, t = tf.transform.rotation, tf.transform.translation
        pts = transform_points(pts, quat_to_rot(q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
        self.map.insert(pts)
        self.used += 1

    def on_cloud_level(self, msg):
        """x, y, yaw from TF (SLAM-corrected, map frame); height and tilt from PX4; floor snap."""
        if self.px4 is None:
            self.get_logger().warn('waiting for PX4 odometry on /fmu/out/vehicle_odometry',
                                   throttle_duration_sec=5.0)
            return
        try:
            # latest pose, no waiting: the cloud stamp is usually a few ms newer than the TF, and
            # waiting for it blocked this node (96% of scans were dropped in the first Gazebo run)
            tf_mb = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            tf_bs = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except Exception as e:  # noqa: BLE001
            self.tf_fail += 1
            self.get_logger().warn(f'TF {self.map_frame} <- {msg.header.frame_id} not available: {e}',
                                   throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = filter_sensor_points(pts.astype(np.float64), self.min_range, self.max_range)
        if len(pts) == 0:
            return
        q, t = tf_bs.transform.rotation, tf_bs.transform.translation
        pts = transform_points(pts, quat_to_rot(q.x, q.y, q.z, q.w), (t.x, t.y, t.z))   # -> base
        # attitude at the scan time, not the newest one: 1 deg of tilt = 9 cm at 5 m
        ts = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        _, alt, roll, pitch = min(self.px4_hist, key=lambda h: abs(h[0] - ts))
        pts = pts @ level_rotation(roll, pitch).T                                        # -> level
        q, t = tf_mb.transform.rotation, tf_mb.transform.translation
        pts = transform_points(pts, yaw_rotation(yaw_from_quat(q.x, q.y, q.z, q.w)),
                               (t.x, t.y, alt))                                          # -> map
        if self.floor_snap:
            f = floor_offset(pts, (t.x, t.y))
            if f is not None:          # use this scan's own floor; keep the last one if none seen
                self.floor_corr = f
                self.floor_seen += 1
            pts[:, 2] -= self.floor_corr
        self.map.insert(pts)
        self.used += 1

    def current_cloud(self):
        pts = self.map.points(self.min_hits)
        return pts, height_colors(pts[:, 2], *self.cz)

    def publish(self):
        pts, rgb = self.current_cloud()
        keep = pts[:, 2] >= self.pub_min_z
        pts, rgb = pts[keep], rgb[keep]
        msg = PointCloud2()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                      for n, o in (('x', 0), ('y', 4), ('z', 8))]
        # packed 0x00RRGGBB, typed FLOAT32 like PCL so RViz (RGB8) and Foxglove both read it
        msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step, msg.row_step = 16, 16 * len(pts)
        msg.is_dense = True
        msg.data = pack_xyzrgb(pts, rgb)
        self.pub.publish(msg)
        self.get_logger().info(f'3D map: {len(pts)} voxels shown, {self.used} scans used, '
                               f'{self.tf_fail} TF misses'
                               + (f', floor fix {self.floor_corr:+.2f} m' if self.level else ''),
                               throttle_duration_sec=10.0)

    def on_save(self, _req, res):
        pts, rgb = self.current_cloud()
        os.makedirs(self.save_dir, exist_ok=True)
        path = os.path.join(self.save_dir, time.strftime('map3d_%Y%m%d_%H%M%S.ply'))
        write_ply(path, pts, rgb)
        res.success, res.message = True, f'saved {len(pts)} points to {path}'
        self.get_logger().info(res.message)
        return res

    def on_clear(self, _req, res):
        self.map.clear()
        res.success, res.message = True, '3D map cleared'
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = HeightMapNode()
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
