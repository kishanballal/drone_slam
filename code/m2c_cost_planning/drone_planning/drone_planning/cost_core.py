"""Cost-aware grid planner for the SLAM drone (no ROS imports, unit tested).

The professor's cost function, for one path P:

    J(P) = w_d * D(P)  +  w_r * R(P)  +  w_b(soc) * E(P) / e_ref  +  w_k * K(P)

  D  distance            metres flown
  R  collision risk      integral of costmap risk (0..1) along the path [m]
                         (risk comes from Nav2's inflation layer: 1 next to a wall,
                         falling off exponentially; unknown space gets a fixed risk)
  E  battery energy      Wh, from a simple multirotor model:
                           cruise: P_hover * time, where the drone flies slower
                                   near obstacles (that is what the RPP controller does)
                           turns:  P_hover * (yaw time + braking time) per turn
  K  path complexity     total heading change [rad] + turn_count_cost per turn
  w_b(soc) = w_b * (1 + battery_gain * (1 - soc))
                         the lower the battery, the more energy matters

It is solved with A* on an 8-connected grid. The heuristic is the octile
distance times the cheapest possible cost per metre, so it is admissible
(the path is optimal for J on the grid when epsilon = 1).
"""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np

UNKNOWN = -1
# nav_msgs/OccupancyGrid values published by Nav2 costmaps:
#   0 free, 1..98 inflation, 99 inscribed (robot would touch), 100 lethal, -1 unknown
INSCRIBED = 99
LETHAL = 100

# 8 moves: (dx, dy); index = direction, 45 degrees apart, counter-clockwise from +x
MOVES = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
STEP = [1.0, math.sqrt(2.0)] * 4


@dataclass
class CostParams:
    w_distance: float = 1.0
    w_risk: float = 4.0
    w_battery: float = 1.0
    w_complexity: float = 0.6
    turn_count_cost: float = 0.3      # extra complexity per heading change (rad-equivalent)
    unknown_risk: float = 0.35        # risk given to unknown cells (frontiers are unknown)
    battery_gain: float = 2.0         # w_b is 3x its value at 0 % battery
    # simple x500 energy model
    hover_power_w: float = 180.0      # W, x500 class hover power
    cruise_speed: float = 0.5         # m/s (RPP desired_linear_vel)
    slow_near_obstacles: float = 0.5  # speed factor lost at risk 1
    yaw_rate: float = 0.8             # rad/s (rotate_to_heading_angular_vel)
    brake_time: float = 1.0           # s lost braking + accelerating per turn
    epsilon: float = 1.0              # >1 = faster, slightly sub-optimal (weighted A*)

    @property
    def e_ref(self):
        """Wh per metre in free space; used to express energy in metre-equivalents."""
        return self.hover_power_w / self.cruise_speed / 3600.0


@dataclass
class PlanResult:
    cells: list = field(default_factory=list)   # [(ix, iy), ...] start .. goal
    cost: float = 0.0
    terms: dict = field(default_factory=dict)    # D, R, E_wh, K, turns, J and weighted parts
    expanded: int = 0
    error: str = ''


def downsample(grid, factor):
    """Block-max a costmap (int array, -1 unknown). A block is unknown only if all of it is."""
    if factor <= 1:
        return grid.astype(np.int16)
    h, w = grid.shape
    H, W = h // factor, w // factor
    g = grid[:H * factor, :W * factor].astype(np.int16)
    return g.reshape(H, factor, W, factor).max(axis=(1, 3))


def risk_of(v, p):
    if v < 0:
        return p.unknown_risk
    return min(1.0, v / 98.0)


def turn_angle(d0, d1):
    """Heading change between two move directions, in radians (0, 45, 90, 135, 180 deg)."""
    if d0 is None:
        return 0.0
    k = abs(d0 - d1) % 8
    return min(k, 8 - k) * math.pi / 4.0


def soc_weight(p, soc):
    soc = 1.0 if soc is None else max(0.0, min(1.0, soc))
    return p.w_battery * (1.0 + p.battery_gain * (1.0 - soc))


def step_terms(v, step_m, dtheta, p):
    """(D, R, E_wh, K, turn) contributed by one grid move into a cell of value v."""
    r = risk_of(v, p)
    speed = p.cruise_speed * (1.0 - p.slow_near_obstacles * r)
    e = p.hover_power_w * step_m / speed / 3600.0
    turned = dtheta > 1e-6
    if turned:
        e += p.hover_power_w * (dtheta / p.yaw_rate + p.brake_time) / 3600.0
    k = dtheta + (p.turn_count_cost if turned else 0.0)
    return step_m, step_m * r, e, k, 1 if turned else 0


def heading_to_dir(yaw):
    if yaw is None:
        return None
    return int(round(yaw / (math.pi / 4.0))) % 8


def nearest_free(grid, cell, radius_cells, block_at=INSCRIBED):
    """Closest cell to `cell` with cost below block_at, within radius; None if none."""
    h, w = grid.shape
    cx, cy = cell
    best, bd = None, None
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            x, y = cx + dx, cy + dy
            if 0 <= x < w and 0 <= y < h and grid[y, x] < block_at:
                d = dx * dx + dy * dy
                if bd is None or d < bd:
                    best, bd = (x, y), d
    return best


def plan(grid, res, start, goal, p, soc=None, start_yaw=None,
         start_escape_cells=4, goal_tolerance_cells=5, max_expansions=400000,
         block_at=INSCRIBED):
    """A* on `grid` (rows = y, cols = x). start/goal are (ix, iy) cells.

    Cells with value >= block_at are blocked (block_at < INSCRIBED keeps a safety
    margin from walls, e.g. through doorways). Near the start (so the drone can leave
    a spot close to a wall) and near the goal only INSCRIBED/LETHAL cells block.
    """
    h, w = grid.shape
    out = PlanResult()
    sx, sy = start
    if not (0 <= sx < w and 0 <= sy < h):
        out.error = 'start outside map'
        return out
    gx, gy = goal
    if not (0 <= gx < w and 0 <= gy < h):
        out.error = 'goal outside map'
        return out
    if grid[gy, gx] >= block_at:
        g2 = nearest_free(grid, goal, goal_tolerance_cells, block_at)
        if g2 is None and block_at < INSCRIBED and grid[gy, gx] < INSCRIBED:
            g2 = (gx, gy)
        if g2 is None:
            g2 = nearest_free(grid, goal, goal_tolerance_cells)
        if g2 is None:
            out.error = 'goal occupied'
            return out
        gx, gy = g2

    wb = soc_weight(p, soc)
    res = float(res)
    # cheapest cost per metre (free cell, no turn): w_d + wb * 1.0 (energy == e_ref)
    h_per_m = (p.w_distance + wb) * p.epsilon
    sqrt2 = math.sqrt(2.0)
    esc2 = start_escape_cells * start_escape_cells

    def heur(x, y):
        dx, dy = abs(x - gx), abs(y - gy)
        return h_per_m * res * (max(dx, dy) + (sqrt2 - 1.0) * min(dx, dy))

    d_start = heading_to_dir(start_yaw)
    INF = float('inf')
    g = {(sx, sy): 0.0}
    came = {(sx, sy): (None, d_start)}   # cell -> (parent, arrival dir)
    closed = set()
    openq = [(heur(sx, sy), 0.0, sx, sy)]
    expanded = 0
    e_ref = p.e_ref
    while openq:
        _, gc, x, y = heapq.heappop(openq)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        expanded += 1
        if (x, y) == (gx, gy):
            break
        if expanded > max_expansions:
            out.error = 'too many expansions'
            out.expanded = expanded
            return out
        d_in = came[(x, y)][1]
        for d, (mx, my) in enumerate(MOVES):
            nx, ny = x + mx, y + my
            if not (0 <= nx < w and 0 <= ny < h) or (nx, ny) in closed:
                continue
            v = int(grid[ny, nx])
            if v >= block_at:
                near_start = (nx - sx) ** 2 + (ny - sy) ** 2 <= esc2
                near_goal = (nx - gx) ** 2 + (ny - gy) ** 2 <= esc2
                if v >= LETHAL or not (near_start or near_goal):
                    continue
                if v >= INSCRIBED and not near_start:
                    continue
            if mx and my:   # no corner cutting past blocked cells
                if grid[y, nx] >= LETHAL or grid[ny, x] >= LETHAL:
                    continue
            D, R, E, K, _ = step_terms(v, STEP[d] * res, turn_angle(d_in, d), p)
            ng = gc + p.w_distance * D + p.w_risk * R + wb * E / e_ref + p.w_complexity * K
            if ng < g.get((nx, ny), INF):
                g[(nx, ny)] = ng
                came[(nx, ny)] = ((x, y), d)
                heapq.heappush(openq, (ng + heur(nx, ny), ng, nx, ny))
    out.expanded = expanded
    if (gx, gy) not in closed:
        out.error = 'no valid path'
        return out
    cells = []
    c = (gx, gy)
    while c is not None:
        cells.append(c)
        c = came[c][0]
    cells.reverse()
    out.cells = cells
    out.terms = evaluate(grid, res, cells, p, soc, start_yaw)
    out.cost = out.terms['J']
    return out


def evaluate(grid, res, cells, p, soc=None, start_yaw=None):
    """Break a cell path down into the four cost terms (for logging and the report)."""
    D = R = E = K = 0.0
    turns = 0
    d_prev = heading_to_dir(start_yaw)
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        d = MOVES.index((x1 - x0, y1 - y0))
        dD, dR, dE, dK, t = step_terms(int(grid[y1, x1]), STEP[d] * res, turn_angle(d_prev, d), p)
        D += dD
        R += dR
        E += dE
        K += dK
        turns += t
        d_prev = d
    wb = soc_weight(p, soc)
    parts = {
        'distance': p.w_distance * D,
        'risk': p.w_risk * R,
        'battery': wb * E / p.e_ref,
        'complexity': p.w_complexity * K,
    }
    return {'D_m': D, 'R_m': R, 'E_Wh': E, 'K_rad': K, 'turns': turns,
            'w_battery_eff': wb, 'parts': parts, 'J': sum(parts.values())}


def path_blocked(grid, cells, skip_first=3):
    """First cell of an old path that is now lethal (a new obstacle), or None."""
    h, w = grid.shape
    for x, y in cells[skip_first:]:
        if 0 <= x < w and 0 <= y < h and grid[y, x] >= LETHAL:
            return (x, y)
    return None


def paths_differ(a, b, res, horizon_m=3.0, tol_m=0.6):
    """True if two cell paths split by more than tol within the first horizon metres."""
    if not a or not b:
        return True
    n = max(1, int(horizon_m / res))
    tol2 = (tol_m / res) ** 2
    bb = b[:n * 2]
    for x, y in a[:n]:
        if min((x - u) ** 2 + (y - v) ** 2 for u, v in bb) > tol2:
            return True
    return False
