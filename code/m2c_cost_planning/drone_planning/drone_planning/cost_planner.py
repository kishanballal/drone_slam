"""cost_planner: Nav2-compatible global planner with the professor's cost function.

    J = w_d * distance + w_r * collision_risk + w_b(battery) * energy + w_k * path_complexity

It serves the same action Nav2's planner_server serves (nav2_msgs/ComputePathToPose)
under the name /compute_path_to_pose_cost. Our behaviour tree
(behavior_trees/nav_to_pose_cost.xml) calls it twice a second, so the path is
re-planned in real time on the live global costmap: when a moving obstacle
blocks the current path, the next plan goes around it (or around the loop).

Inputs
  /global_costmap/costmap      nav_msgs/OccupancyGrid  (Nav2, live LiDAR obstacles + SLAM map)
  /fmu/out/battery_status_v1   px4_msgs/BatteryStatus  (state of charge, optional)
  TF map -> base_link          start pose + heading
Outputs
  /cost_planner/path           nav_msgs/Path           (for RViz)
  /cost_planner/cost           std_msgs/String (JSON)  cost terms of the latest plan
  /cost_planner/blocked        visualization_msgs/Marker  where a new obstacle cut the old path

Weights can be changed while flying, for example:
  ros2 param set /cost_planner w_risk 8.0
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from drone_planning.cost_core import (CostParams, downsample, evaluate, path_blocked,
                                      paths_differ, plan)

try:
    from px4_msgs.msg import BatteryStatus
except ImportError:            # planner still works, battery term uses 100 %
    BatteryStatus = None

R = ComputePathToPose.Result
ERR = {k: getattr(R, k, v) for k, v in [
    ('NONE', 0), ('UNKNOWN', 300), ('TF_ERROR', 302), ('START_OUTSIDE_MAP', 303),
    ('GOAL_OUTSIDE_MAP', 304), ('GOAL_OCCUPIED', 306), ('NO_VALID_PATH', 308)]}
ERROR_MAP = {'start outside map': 'START_OUTSIDE_MAP', 'goal outside map': 'GOAL_OUTSIDE_MAP',
             'goal occupied': 'GOAL_OCCUPIED', 'no valid path': 'NO_VALID_PATH',
             'too many expansions': 'NO_VALID_PATH'}

WEIGHT_PARAMS = ['w_distance', 'w_risk', 'w_battery', 'w_complexity', 'turn_count_cost',
                 'unknown_risk', 'battery_gain', 'hover_power_w', 'cruise_speed',
                 'slow_near_obstacles', 'yaw_rate', 'brake_time', 'epsilon']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CostPlanner(Node):
    def __init__(self):
        super().__init__('cost_planner')
        d = CostParams()
        for k in WEIGHT_PARAMS:
            self.declare_parameter(k, float(getattr(d, k)))
        self.declare_parameter('action_name', 'compute_path_to_pose_cost')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('battery_topic', '/fmu/out/battery_status_v1')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('plan_resolution', 0.1)     # m; costmap is block-maxed to this
        self.declare_parameter('goal_tolerance', 0.5)      # m; goal inside an obstacle -> nearest free
        # Hard safety margin: cells with costmap value >= this are not flown through
        # (45 = about 0.7 m from a wall with our inflation). Falls back to 99 if no path.
        self.declare_parameter('clearance_cost', 45)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.robot_frame = g('robot_frame')
        self.plan_res = float(g('plan_resolution'))

        self.lock = threading.Lock()
        self.grid = None          # downsampled int16 grid
        self.grid_info = None     # (origin_x, origin_y, res, frame)
        self.soc = None
        self.last = None          # dict: goal xy, path world xy list, J
        self.replans = 0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, g('costmap_topic'), self.on_costmap, latched)
        if BatteryStatus is not None:
            be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(BatteryStatus, g('battery_topic'), self.on_battery, be)
        self.pub_path = self.create_publisher(Path, '/cost_planner/path', 1)
        self.pub_cost = self.create_publisher(String, '/cost_planner/cost', 1)
        self.pub_mark = self.create_publisher(Marker, '/cost_planner/blocked', 1)

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.server = ActionServer(
            self, ComputePathToPose, g('action_name'), execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'cost_planner up: action /{g("action_name")}, costmap {g("costmap_topic")}, '
            f'battery {g("battery_topic") if BatteryStatus else "(px4_msgs missing, 100 %)"}')

    # ------------------------------------------------------------------ inputs
    def on_costmap(self, msg):
        info = msg.info
        raw = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        f = max(1, int(round(self.plan_res / info.resolution)))
        grid = downsample(raw, f)
        with self.lock:
            self.grid = grid
            self.grid_info = (info.origin.position.x, info.origin.position.y,
                              info.resolution * f, msg.header.frame_id)

    def on_battery(self, msg):
        if msg.remaining >= 0.0:
            self.soc = float(msg.remaining)

    def params(self):
        return CostParams(**{k: float(self.get_parameter(k).value) for k in WEIGHT_PARAMS})

    # ------------------------------------------------------------------ helpers
    def pose_in(self, frame, pose_stamped=None):
        """(x, y, yaw) of pose_stamped (or of the robot if None) in `frame`, 2D."""
        if pose_stamped is None:
            src, x, y, yaw = self.robot_frame, 0.0, 0.0, 0.0
        else:
            src = pose_stamped.header.frame_id or frame
            p = pose_stamped.pose
            x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if src == frame:
            return x, y, yaw
        t = self.tf.lookup_transform(frame, src, Time(), timeout=Duration(seconds=0.5))
        tr, a = t.transform.translation, yaw_of(t.transform.rotation)
        c, s = math.cos(a), math.sin(a)
        return tr.x + c * x - s * y, tr.y + s * x + c * y, yaw + a

    @staticmethod
    def to_cell(info, x, y):
        ox, oy, res, _ = info
        return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))

    @staticmethod
    def to_world(info, c):
        ox, oy, res, _ = info
        return ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res

    def fail(self, goal_handle, key, text):
        self.get_logger().warn(f'plan failed: {text}', throttle_duration_sec=2.0)
        res = R()
        res.error_code = ERR[key]
        if hasattr(res, 'error_msg'):
            res.error_msg = text
        goal_handle.abort()
        return res

    # ------------------------------------------------------------------ action
    def execute(self, goal_handle):
        t0 = time.time()
        req = goal_handle.request
        with self.lock:
            grid, info = self.grid, self.grid_info
        if grid is None:
            return self.fail(goal_handle, 'UNKNOWN', 'no costmap received yet')
        frame = info[3]
        try:
            sx, sy, syaw = self.pose_in(frame, req.start if req.use_start else None)
            gx, gy, _ = self.pose_in(frame, req.goal)
        except Exception as e:  # noqa: BLE001  (tf2 raises several types)
            return self.fail(goal_handle, 'TF_ERROR', f'TF: {e}')

        p = self.params()
        tol = max(1, int(self.get_parameter('goal_tolerance').value / info[2]))
        clear = int(self.get_parameter('clearance_cost').value)
        args = (grid, info[2], self.to_cell(info, sx, sy), self.to_cell(info, gx, gy), p)
        r = plan(*args, soc=self.soc, start_yaw=syaw, goal_tolerance_cells=tol, block_at=clear)
        if r.error and clear < 99:
            r = plan(*args, soc=self.soc, start_yaw=syaw, goal_tolerance_cells=tol)
            if not r.error:
                self.get_logger().info('no path with the full wall margin; using the tight one',
                                       throttle_duration_sec=5.0)
        if r.error:
            return self.fail(goal_handle, ERROR_MAP.get(r.error, 'UNKNOWN'),
                             f'{r.error} (goal {gx:.1f}, {gy:.1f})')

        xy = [self.to_world(info, c) for c in r.cells]
        xy[0] = (sx, sy)
        if math.hypot(xy[-1][0] - gx, xy[-1][1] - gy) < 1.5 * info[2]:
            xy[-1] = (gx, gy)
        self.report(grid, info, p, r, xy, (gx, gy), time.time() - t0)

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(xy):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            nx, ny = xy[min(i + 1, len(xy) - 1)]
            px, py = xy[max(i - 1, 0)]
            a = math.atan2(ny - py, nx - px)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
            path.poses.append(ps)
        self.pub_path.publish(path)

        res = R()
        res.path = path
        dt = time.time() - t0
        res.planning_time.sec = int(dt)
        res.planning_time.nanosec = int((dt % 1.0) * 1e9)
        res.error_code = ERR['NONE']
        goal_handle.succeed()
        return res

    # ------------------------------------------------------------------ logging
    def report(self, grid, info, p, r, xy, goal, dt):
        t = r.terms
        msg = {k: round(v, 3) if isinstance(v, float) else v for k, v in t.items() if k != 'parts'}
        msg['parts'] = {k: round(v, 2) for k, v in t['parts'].items()}
        msg['soc'] = None if self.soc is None else round(self.soc, 3)
        msg['plan_ms'] = round(dt * 1000.0, 1)
        msg['replans'] = self.replans
        self.pub_cost.publish(String(data=json.dumps(msg)))

        line = (f'D={t["D_m"]:.1f} m  risk={t["R_m"]:.2f}  E={t["E_Wh"]:.2f} Wh  '
                f'K={t["K_rad"]:.1f} rad ({t["turns"]} turns)  J={t["J"]:.1f}  '
                f'battery={"?" if self.soc is None else f"{self.soc * 100:.0f}%"}  '
                f'{dt * 1000:.0f} ms')
        last = self.last
        same_goal = last is not None and math.hypot(goal[0] - last['goal'][0],
                                                    goal[1] - last['goal'][1]) < 0.5
        if not same_goal:
            self.get_logger().info(f'NEW GOAL ({goal[0]:.1f}, {goal[1]:.1f}): {line}')
        else:
            old_cells = [self.to_cell(info, x, y) for x, y in last['xy']]
            # only the part of the old path still ahead of the drone matters
            k = min(range(len(old_cells)), key=lambda i: (old_cells[i][0] - r.cells[0][0]) ** 2
                    + (old_cells[i][1] - r.cells[0][1]) ** 2)
            ahead = old_cells[k:]
            hit = path_blocked(grid, ahead)
            if hit is not None:
                self.replans += 1
                hx, hy = self.to_world(info, hit)
                old_j = evaluate(grid, info[2], ahead, p, self.soc)['J'] if len(ahead) > 1 else 0
                self.get_logger().warn(
                    f'REPLAN #{self.replans}: obstacle on the old path at ({hx:.1f}, {hy:.1f}) '
                    f'(old J={old_j:.1f} incl. collision) -> new path {line}')
                self.mark(info[3], hx, hy)
            elif paths_differ(ahead, r.cells, info[2]):
                self.get_logger().info(f'route changed (cheaper path found): {line}')
        self.last = {'goal': goal, 'xy': xy, 'J': t['J']}

    def mark(self, frame, x, y):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'blocked', 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.6
        m.color.r, m.color.a = 1.0, 0.8
        m.lifetime = Duration(seconds=3.0).to_msg()
        self.pub_mark.publish(m)


def main():
    rclpy.init()
    node = CostPlanner()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
