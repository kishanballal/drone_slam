#!/usr/bin/env python3
"""Fallback frontier explorer (Milestone 2b plan B, no Nav2).

Every `replan_period` seconds it:
  1. reads the slam_toolbox /map and the drone pose (TF map -> base_link),
  2. inflates walls by `inflate_radius` and runs a BFS from the drone over free space,
  3. finds reachable frontier clusters (free cells that touch unknown space),
  4. keeps the current goal while it is still a frontier, otherwise picks the
     nearest one (by path length),
  5. publishes a "carrot" up to `lookahead` metres along the BFS path (only as
     far as a straight line stays in free space) on /drone/position_target.

cmd_vel_bridge turns that target into a PX4 position setpoint. When no frontier
is left the drone flies back to where exploration started and hovers.
Goals that are reached but stay frontiers, or that time out, are blacklisted.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import TransformException

from drone_explore.core import (traversable_mask, bfs, nearest_traversable,
                                frontier_clusters, extract_path, pick_carrot)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.inflate_radius = float(p('inflate_radius', 0.45).value)
        self.min_frontier = float(p('min_frontier_size', 0.5).value)   # metres
        self.lookahead = float(p('lookahead', 1.2).value)
        self.reached_dist = float(p('goal_reached_dist', 0.4).value)
        self.keep_radius = float(p('keep_goal_radius', 0.6).value)
        self.goal_timeout = float(p('goal_timeout', 40.0).value)       # wall-clock s
        self.blacklist_radius = float(p('blacklist_radius', 0.8).value)
        self.return_home = bool(p('return_home', True).value)
        period = float(p('replan_period', 1.0).value)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value,
                                 self.on_map, map_qos)
        self.pub_target = self.create_publisher(PoseStamped, '/drone/position_target', 10)
        self.pub_path = self.create_publisher(Path, '/frontier/path', 10)
        self.pub_marker = self.create_publisher(Marker, '/frontier/goal', 10)
        self.pub_state = self.create_publisher(String, '/frontier/state', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map = None
        self.home = None            # (x, y) in map frame
        self.goal = None            # (x, y) in map frame
        self.goal_since = 0.0
        self.blacklist = []
        self.mode = 'EXPLORE'       # EXPLORE -> RETURN -> DONE
        self.create_timer(period, self.plan)
        self.get_logger().info('Frontier explorer started (fallback mode).')

    def on_map(self, msg):
        self.map = msg

    # ------------------------------------------------------------- helpers
    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {self.map_frame} -> {self.base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        return t.transform.translation.x, t.transform.translation.y

    def blacklisted(self, xy):
        return any(math.hypot(xy[0] - b[0], xy[1] - b[1]) < self.blacklist_radius
                   for b in self.blacklist)

    def say(self, text):
        self.get_logger().info(text)
        self.pub_state.publish(String(data=text))

    # ---------------------------------------------------------------- plan
    def plan(self):
        if self.mode == 'DONE' or self.map is None:
            if self.map is None:
                self.get_logger().info('Waiting for /map ...', throttle_duration_sec=3.0)
            return
        robot = self.robot_xy()
        if robot is None:
            return
        if self.home is None:
            self.home = robot

        info = self.map.info
        res, w, h = info.resolution, info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        grid = np.array(self.map.data, dtype=np.int16).reshape(h, w)

        def to_cell(xy):
            return int((xy[1] - oy) / res), int((xy[0] - ox) / res)

        def to_xy(rc):
            return ox + (rc[1] + 0.5) * res, oy + (rc[0] + 0.5) * res

        t0 = time.monotonic()
        trav, unknown = traversable_mask(grid, int(math.ceil(self.inflate_radius / res)))
        start = nearest_traversable(trav, to_cell(robot), int(1.0 / res))
        if start is None:
            self.get_logger().warn('Drone is not near any free, inflated-safe cell; hovering.',
                                   throttle_duration_sec=2.0)
            return
        dist, parent = bfs(trav, start)

        goal_rc = None
        if self.mode == 'EXPLORE':
            goal_rc = self.choose_frontier(trav, unknown, dist, res, to_xy, robot)
            if goal_rc is None:
                if self.return_home:
                    self.mode = 'RETURN'
                    self.say('No frontiers left: exploration complete, returning home.')
                else:
                    self.mode = 'DONE'
                    self.say('No frontiers left: exploration complete. Hovering.')
                    return
        if self.mode == 'RETURN':
            if math.hypot(robot[0] - self.home[0], robot[1] - self.home[1]) < self.reached_dist:
                self.mode = 'DONE'
                self.say('Back home. Hovering. Land with: '
                         'ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"')
                return
            goal_rc = nearest_traversable(trav, to_cell(self.home), int(1.0 / res))
            if goal_rc is None or dist[goal_rc[0] * w + goal_rc[1]] < 0:
                self.get_logger().warn('Home not reachable on the map; hovering.',
                                       throttle_duration_sec=5.0)
                return

        path = extract_path(parent, w, goal_rc)
        carrot = pick_carrot(trav, path, self.lookahead / res)
        self.publish(carrot, path, goal_rc, to_xy)
        self.get_logger().debug(f'plan took {time.monotonic() - t0:.2f}s')

    def choose_frontier(self, trav, unknown, dist, res, to_xy, robot):
        min_cells = max(1, int(self.min_frontier / res))
        clusters = [c for c in frontier_clusters(trav, unknown, dist, min_cells)
                    if not self.blacklisted(to_xy(c['goal']))]
        now = time.monotonic()

        if self.goal is not None:
            if math.hypot(robot[0] - self.goal[0], robot[1] - self.goal[1]) < self.reached_dist:
                # Reached but the frontier is still there (e.g. unknown space we can't see)
                self.blacklist.append(self.goal)
                self.goal = None
            elif now - self.goal_since > self.goal_timeout:
                self.get_logger().warn(f'Goal {self.goal} timed out; blacklisting it.')
                self.blacklist.append(self.goal)
                self.goal = None
            clusters = [c for c in clusters if not self.blacklisted(to_xy(c['goal']))]

        if not clusters:
            self.goal = None
            return None

        if self.goal is not None:
            near = [c for c in clusters
                    if math.hypot(to_xy(c['goal'])[0] - self.goal[0],
                                  to_xy(c['goal'])[1] - self.goal[1]) < self.keep_radius]
            if near:
                best = min(near, key=lambda c: c['steps'])
                self.goal = to_xy(best['goal'])      # follow it as the map grows
                return best['goal']

        best = min(clusters, key=lambda c: (c['steps'], -c['size']))
        self.goal = to_xy(best['goal'])
        self.goal_since = now
        self.say(f'New frontier goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}), '
                 f'{best["steps"] * res:.1f} m away, {len(clusters)} frontiers left')
        return best['goal']

    def publish(self, carrot, path, goal_rc, to_xy):
        stamp = self.get_clock().now().to_msg()
        t = PoseStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.pose.position.x, t.pose.position.y = to_xy(carrot)
        t.pose.orientation.w = 1.0
        self.pub_target.publish(t)

        pm = Path()
        pm.header = t.header
        for rc in path[::4] + [path[-1]]:
            ps = PoseStamped()
            ps.header = t.header
            ps.pose.position.x, ps.pose.position.y = to_xy(rc)
            ps.pose.orientation.w = 1.0
            pm.poses.append(ps)
        self.pub_path.publish(pm)

        mk = Marker()
        mk.header = t.header
        mk.ns, mk.id, mk.type, mk.action = 'frontier_goal', 0, Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y = to_xy(goal_rc)
        mk.pose.position.z = 1.5
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.4
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.3, 0.0, 0.9
        self.pub_marker.publish(mk)


def main():
    rclpy.init()
    node = FrontierExplorer()
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
