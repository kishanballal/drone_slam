"""M4: the drone plans its own search route from the map it built, then flies it with Nav2.

 - While exploring it records where it flew (TF map -> base_link).
 - On /search_route/start (std_msgs/Empty) it takes the latest /map, splits it into areas,
   picks the fewest stops that let the 3D LiDAR see the floor it has not seen yet,
   orders them shortest-first (start -> each stop once -> home) and flies them.
 - RViz: /search_route/markers (route line + numbered stops, green once reached).
 - Log lines for the demo script: 'SEARCH ROUTE PLAN', 'STOP i/n', 'SEARCH ROUTE DONE'.
 - Saves search_route.txt and search_route.png (areas, stops, route) to out_dir.
"""
import math
import os
import struct
import time
import zlib

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Empty, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from drone_search import route_core as rc


def write_png(path, rgb):
    """Tiny PNG writer (no PIL needed). rgb: HxWx3 uint8, row 0 = top."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


class SearchRoute(Node):
    def __init__(self):
        super().__init__('search_route')
        p = self.declare_parameter
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('plan_cell', 0.2)
        p('see_radius', 5.0)
        p('stop_clearance', 0.8)
        p('travel_clearance', 0.5)
        p('door_split', 1.0)
        p('cover_goal', 0.95)
        p('min_gain_m2', 1.0)
        p('max_stops', 12)
        p('home_xy', [0.0, 0.0])
        p('goal_timeout', 120.0)
        p('pause_at_stop', 2.0)
        p('record_period', 1.0)
        p('record_spacing', 0.5)
        p('fixed_tour_xy', [0.0])          # old hand-written tour, only for the comparison print
        p('out_dir', os.path.expanduser('~/slam_drone/maps'))
        g = lambda n: self.get_parameter(n).value
        self.P = {n: g(n) for n in ['map_frame', 'base_frame', 'plan_cell', 'see_radius',
                                    'stop_clearance', 'travel_clearance', 'door_split',
                                    'cover_goal', 'min_gain_m2', 'max_stops', 'home_xy',
                                    'goal_timeout', 'pause_at_stop', 'record_spacing',
                                    'fixed_tour_xy', 'out_dir']}

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, latched)
        self.create_subscription(Empty, '/search_route/start', self.on_start, 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/search_route/markers', latched)
        self.pub_status = self.create_publisher(String, '/search_route/status', latched)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.flown = []            # (x, y) samples, >= record_spacing apart
        self.state = 'record'
        self.route = None
        self.goal_handle = None
        self.seq = 0
        self.idx = 0
        self.t_goal = 0.0
        self.t_pause = 0.0
        self.reached = []
        self.t_route0 = 0.0
        self.flown_route_start = 0
        self.create_timer(g('record_period'), self.record)
        self.create_timer(0.5, self.step)
        self.get_logger().info('search_route ready: recording the flight path, '
                               'waiting for /search_route/start')

    # ---------------------------------------------------------------------
    def on_map(self, msg):
        self.map = msg

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.P['map_frame'], self.P['base_frame'], Time(),
                                         timeout=Duration(seconds=0.2))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def record(self):
        xy = self.pose()
        if xy is None:
            return
        if not self.flown or math.dist(xy, self.flown[-1]) >= self.P['record_spacing']:
            self.flown.append(xy)

    def say(self, text):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=text))

    # ---------------------------------------------------------------------
    def on_start(self, _msg):
        if self.state != 'record':
            self.get_logger().debug('route already started')
            return
        if self.map is None:
            self.get_logger().error('no /map yet: cannot plan')
            return
        start = self.pose() or (self.flown[-1] if self.flown else tuple(self.P['home_xy']))
        m = self.map
        t0 = time.time()
        grid = rc.Grid(m.data, m.info.width, m.info.height, m.info.resolution,
                       (m.info.origin.position.x, m.info.origin.position.y), self.P['plan_cell'])
        res = rc.plan_route(grid, start, tuple(self.P['home_xy']), self.flown,
                            see_radius=self.P['see_radius'], stop_clear=self.P['stop_clearance'],
                            travel_clear=self.P['travel_clearance'], door_split=self.P['door_split'],
                            cover_goal=self.P['cover_goal'], min_gain_m2=self.P['min_gain_m2'],
                            max_stops=self.P['max_stops'])
        dt = time.time() - t0
        fx = list(self.P['fixed_tour_xy'])
        cmp_txt = ''
        if len(fx) >= 4 and len(fx) % 2 == 0:
            pts = list(zip(fx[0::2], fx[1::2]))
            L0 = rc.fixed_tour_length(grid, start, pts, self.P['travel_clearance'])
            if math.isfinite(L0) and L0 > 0:
                cmp_txt = (f' | old fixed {len(pts)}-stop tour from here: {L0:.1f} m '
                           f'-> {100.0 * (1.0 - res["length"] / L0):.0f}% shorter')
        self.route = res
        n = len(res['stops'])
        self.say(f'SEARCH ROUTE PLAN: map split into {res["n_areas"]} areas, '
                 f'{100 * res["seen_while_exploring"]:.0f}% of the floor already seen while exploring, '
                 f'{n} stops cover {100 * res["covered_fraction"]:.0f}% of the rest, '
                 f'route {res["length"]:.1f} m (planned in {dt:.1f} s){cmp_txt}')
        for i, (x, y, a) in enumerate(res['stops']):
            self.get_logger().info(f'  stop {i + 1}: ({x:.1f}, {y:.1f}) in area {a}')
        self.goals = [(x, y, f'stop {i + 1}/{n} (area {a})') for i, (x, y, a) in enumerate(res['stops'])]
        self.goals.append((*res['home'], 'home'))
        self.reached = [False] * len(self.goals)
        self.publish_markers()
        self.save_files(grid, res, start, cmp_txt)
        self.idx = 0
        self.t_route0 = time.time()
        self.flown_route_start = len(self.flown)
        self.state = 'send'

    # ---------------------------------------------------------------------
    def step(self):
        if self.state == 'send':
            if self.idx >= len(self.goals):
                self.finish()
                return
            if not self.nav.wait_for_server(timeout_sec=0.1):
                return
            x, y, name = self.goals[self.idx]
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = self.P['map_frame']
            g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
            g.pose.pose.orientation.w = 1.0
            self.get_logger().info(f'== {name}: ({x:.1f}, {y:.1f}) ==')
            self.state = 'wait'
            self.t_goal = time.time()
            self.seq += 1
            seq = self.seq
            self.nav.send_goal_async(g).add_done_callback(lambda f: self.on_goal_response(f, seq))
        elif self.state == 'wait':
            if time.time() - self.t_goal > self.P['goal_timeout']:
                self.get_logger().warn('goal timed out, skipping to the next stop')
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.seq += 1           # ignore any late answer for the old goal
                self.next_goal(False)
        elif self.state == 'pause' and time.time() >= self.t_pause:
            self.state = 'send'

    def on_goal_response(self, fut, seq):
        if seq != self.seq:
            return
        h = fut.result()
        if not h.accepted:
            self.get_logger().warn('goal rejected')
            self.next_goal(False)
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, fut, seq):
        if seq != self.seq or self.state != 'wait':
            return
        ok = fut.result().status == GoalStatus.STATUS_SUCCEEDED
        self.goal_handle = None
        self.next_goal(ok)

    def next_goal(self, ok):
        name = self.goals[self.idx][2]
        self.reached[self.idx] = ok
        self.get_logger().info(f'STOP {name}: {"SUCCEEDED" if ok else "FAILED"}')
        self.publish_markers()
        self.idx += 1
        self.t_pause = time.time() + self.P['pause_at_stop']
        self.state = 'pause'

    def finish(self):
        self.state = 'done'
        path = self.flown[max(0, self.flown_route_start - 1):]
        flown_m = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        n_ok = sum(self.reached[:-1])
        txt = (f'SEARCH ROUTE DONE: {n_ok}/{len(self.goals) - 1} stops reached, home '
               f'{"reached" if self.reached[-1] else "NOT reached"}, flew {flown_m:.1f} m '
               f'in {time.time() - self.t_route0:.0f} s')
        self.say(txt)
        try:
            with open(os.path.join(self.P['out_dir'], 'search_route.txt'), 'a') as f:
                f.write(txt + '\n')
        except OSError:
            pass

    # ---------------------------------------------------------------------
    def publish_markers(self):
        res = self.route
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        clear = Marker(action=Marker.DELETEALL)
        ma.markers.append(clear)
        line = Marker(type=Marker.LINE_STRIP, ns='route', id=0)
        line.header.frame_id, line.header.stamp = self.P['map_frame'], now
        line.scale.x = 0.08
        line.color = ColorRGBA(r=0.1, g=0.9, b=1.0, a=0.9)
        line.pose.orientation.w = 1.0
        line.points = [Point(x=x, y=y, z=1.5) for x, y in res['path_xy']]
        ma.markers.append(line)
        for i, (x, y, _name) in enumerate(self.goals):
            done = self.reached[i]
            s = Marker(type=Marker.SPHERE, ns='stops', id=i)
            s.header.frame_id, s.header.stamp = self.P['map_frame'], now
            s.pose.position = Point(x=x, y=y, z=1.5)
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.45
            s.color = (ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0) if done
                       else ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0))
            ma.markers.append(s)
            t = Marker(type=Marker.TEXT_VIEW_FACING, ns='labels', id=i)
            t.header.frame_id, t.header.stamp = self.P['map_frame'], now
            t.pose.position = Point(x=x, y=y, z=2.1)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.5
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = 'H' if i == len(self.goals) - 1 else str(i + 1)
            ma.markers.append(t)
        self.pub_mk.publish(ma)

    def save_files(self, grid, res, start, cmp_txt):
        d = self.P['out_dir']
        try:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, 'search_route.txt'), 'w') as f:
                f.write(f'start ({start[0]:.2f}, {start[1]:.2f})\n')
                f.write(f'areas {res["n_areas"]}: ' + ', '.join(
                    f'area {k} {v:.0f} m2' for k, v in res['areas_m2'].items()) + '\n')
                f.write(f'seen while exploring {100 * res["seen_while_exploring"]:.0f}%\n')
                for i, (x, y, a) in enumerate(res['stops']):
                    f.write(f'stop {i + 1}: ({x:.2f}, {y:.2f}) area {a}\n')
                f.write(f'route length {res["length"]:.1f} m{cmp_txt}\n')
            pal = np.array([[0, 0, 0], [255, 150, 150], [150, 230, 150], [150, 170, 255],
                            [250, 230, 120], [120, 230, 230], [230, 150, 230]], np.uint8)
            lab = res['labels']
            img = pal[np.where(lab > 0, (lab - 1) % 6 + 1, 0)]
            img[grid.cls == rc.UNK] = (40, 40, 40)
            img[grid.cls == rc.OCC] = (90, 90, 90)
            for x, y in self.flown:
                r, c = grid.to_cell(x, y)
                if grid.inside(r, c):
                    img[r, c] = (160, 160, 160)
            for x, y in res['path_xy']:
                r, c = grid.to_cell(x, y)
                img[r, c] = (255, 255, 255)
            for x, y, _a in res['stops']:
                r, c = grid.to_cell(x, y)
                img[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = (230, 60, 0)
            big = np.repeat(np.repeat(img[::-1], 6, axis=0), 6, axis=1)
            write_png(os.path.join(d, 'search_route.png'), np.ascontiguousarray(big))
        except Exception as e:      # never let a file problem stop the flight
            self.get_logger().warn(f'could not save route files: {e}')


def main():
    rclpy.init()
    n = SearchRoute()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
