"""obstacle_mover: makes the spawned "walker" obstacles pace up and down.

Each walker is a 0.5 m x 2.0 m cylinder (models/walker) with Gazebo's
VelocityControl system, so it moves at exactly the commanded speed. This node
sends /model/<name>/cmd_vel (bridged to Gazebo) and turns each walker around
after it has travelled `lengths[i]` metres, forever.

Params (lists, one entry per walker):
  names    ['walker_1', 'walker_2']
  axes     ['x', 'y']          world axis it paces along
  lengths  [8.0, 3.0]          metres from its spawn point (positive direction first)
  speeds   [0.4, 0.3]          m/s
  pause    1.0                 s standing still at each end
  start_delay 0.0              s before the first move (e.g. let the drone take off)
  starts_x, starts_y           spawn points (world = map frame, the drone starts at 0, 0)
  stop_dist   1.4              a walker heading towards the drone waits while the drone
                               is closer than this (people step aside; the sim walkers
                               have no eyes, and a hit would end the demo, not test planning)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        self.declare_parameter('names', ['walker_1', 'walker_2'])
        self.declare_parameter('axes', ['x', 'y'])
        self.declare_parameter('lengths', [8.0, 3.0])
        self.declare_parameter('speeds', [0.4, 0.3])
        self.declare_parameter('pause', 1.0)
        self.declare_parameter('start_delay', 0.0)
        self.declare_parameter('starts_x', [0.0])
        self.declare_parameter('starts_y', [0.0])
        self.declare_parameter('stop_dist', 1.4)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        names, axes = list(g('names')), list(g('axes'))
        lengths, speeds = list(g('lengths')), list(g('speeds'))
        if not (len(names) == len(axes) == len(lengths) == len(speeds)):
            raise ValueError('names, axes, lengths and speeds must have the same length')
        self.pause = float(g('pause'))
        sx, sy = list(g('starts_x')), list(g('starts_y'))
        self.stop_dist = float(g('stop_dist'))
        self.frames = (g('map_frame'), g('base_frame'))
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.walkers = []
        for i, (n, a, L, v) in enumerate(zip(names, axes, lengths, speeds)):
            self.walkers.append({
                'name': n, 'axis': a, 'len': float(L), 'v': float(v),
                'pos': 0.0, 'dir': 1.0, 'wait': 0.0, 'held': False,
                'x0': float(sx[i]) if i < len(sx) else None,
                'y0': float(sy[i]) if i < len(sy) else None,
                'pub': self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10)})
        self.t_start = None
        self.delay = float(g('start_delay'))
        self.last = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info('moving: ' + ', '.join(
            f'{w["name"]} {w["len"]:.1f} m along {w["axis"]} at {w["v"]:.2f} m/s'
            for w in self.walkers))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start, self.last = now, now
            return
        dt = now - self.last
        self.last = now
        if dt <= 0.0 or dt > 1.0:          # sim paused / clock jump: don't integrate
            return
        moving = now - self.t_start >= self.delay
        drone = self.drone_xy()
        for w in self.walkers:
            cmd = Twist()
            if moving and self.must_wait(w, drone):
                pass                       # hold still until the drone has passed
            elif moving:
                if w['wait'] > 0.0:
                    w['wait'] -= dt
                else:
                    w['pos'] += w['dir'] * w['v'] * dt
                    if w['pos'] >= w['len'] or w['pos'] <= 0.0:
                        w['pos'] = min(max(w['pos'], 0.0), w['len'])
                        w['dir'] *= -1.0
                        w['wait'] = self.pause
                    else:
                        setattr(cmd.linear, w['axis'], w['dir'] * w['v'])
            w['pub'].publish(cmd)

    def drone_xy(self):
        try:
            t = self.tf.lookup_transform(self.frames[0], self.frames[1], Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def must_wait(self, w, drone):
        if drone is None or w['x0'] is None or self.stop_dist <= 0.0:
            return False
        x, y = w['x0'], w['y0']
        if w['axis'] == 'x':
            x += w['pos']
            ahead = (drone[0] - x) * w['dir']
        else:
            y += w['pos']
            ahead = (drone[1] - y) * w['dir']
        close = math.hypot(drone[0] - x, drone[1] - y) < self.stop_dist and ahead > -0.2
        if close != w['held']:
            w['held'] = close
            self.get_logger().info(f'{w["name"]} ' + ('waits: drone close ahead' if close else 'walks on'))
        return close


def main():
    rclpy.init()
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for w in node.walkers:            # stop them where they are
            w['pub'].publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
