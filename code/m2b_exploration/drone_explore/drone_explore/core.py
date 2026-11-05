"""Pure-Python helpers for M2b (no ROS imports, so they can be unit tested).

Frame conventions used everywhere in this package
-------------------------------------------------
ROS side : ENU world frames (map, odom) and FLU body frame (base_link).
           Yaw is measured from +x, counter-clockwise positive.
PX4 side : NED local frame and FRD body frame.
           PX4 "heading" is measured from north, clockwise positive.
"""
import math
from collections import deque

import numpy as np


# ----------------------------------------------------------------------------
# Small math helpers
# ----------------------------------------------------------------------------
def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def clip_norm(x, y, lim):
    """Scale the 2D vector (x, y) down so its length is at most lim."""
    n = math.hypot(x, y)
    if n > lim > 0.0:
        return x * lim / n, y * lim / n
    return x, y


def yaw_from_quat(x, y, z, w):
    """Yaw (rotation about z) of a quaternion, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_to_ned(fwd, left, heading):
    """Rotate a horizontal FLU body vector into PX4 local NED.

    fwd, left : body-frame components (ROS FLU: x forward, y left)
    heading   : PX4 heading in rad (0 = facing north, clockwise positive)
    returns   : (north, east)
    """
    c, s = math.cos(heading), math.sin(heading)
    north = fwd * c + left * s
    east = fwd * s - left * c
    return north, east


def world_delta_to_body(dx, dy, yaw):
    """Rotate a horizontal ENU world vector into the FLU body frame (yaw = ENU yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    fwd = c * dx + s * dy
    left = -s * dx + c * dy
    return fwd, left


# ----------------------------------------------------------------------------
# Occupancy-grid helpers for the fallback frontier explorer
# ----------------------------------------------------------------------------
OCC_THRESHOLD = 65   # slam_toolbox publishes -1 (unknown), 0 (free), 100 (occupied)


def dilate(mask, r):
    """Binary dilation of a bool HxW array with a disk of radius r cells."""
    if r <= 0:
        return mask.copy()
    h, w = mask.shape
    padded = np.pad(mask, r, constant_values=False)
    out = np.zeros_like(mask)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= padded[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


def classify(grid):
    """Split an int8 HxW occupancy grid into (free, occupied, unknown) bool masks."""
    unknown = grid < 0
    occupied = grid >= OCC_THRESHOLD
    free = ~unknown & ~occupied
    return free, occupied, unknown


def traversable_mask(grid, inflate_cells):
    """Free cells at least inflate_cells away from any obstacle. Border cells are
    always blocked so the BFS below never has to bounds-check."""
    free, occupied, unknown = classify(grid)
    trav = free & ~dilate(occupied, inflate_cells)
    trav[0, :] = trav[-1, :] = False
    trav[:, 0] = trav[:, -1] = False
    return trav, unknown


def bfs(trav, start_rc):
    """8-connected BFS over traversable cells.
    Returns (dist, parent) as flat Python lists; dist = -1 means unreachable."""
    h, w = trav.shape
    flat = trav.ravel().tolist()
    dist = [-1] * (h * w)
    parent = [-1] * (h * w)
    s = start_rc[0] * w + start_rc[1]
    dist[s] = 0
    q = deque([s])
    offs = (-w, w, -1, 1, -w - 1, -w + 1, w - 1, w + 1)
    while q:
        c = q.popleft()
        d = dist[c] + 1
        for o in offs:
            n = c + o
            if flat[n] and dist[n] < 0:
                dist[n] = d
                parent[n] = c
                q.append(n)
    return dist, parent


def nearest_traversable(trav, rc, max_cells):
    """Closest traversable cell to rc within a square window, or None."""
    h, w = trav.shape
    r0, c0 = rc
    if 0 <= r0 < h and 0 <= c0 < w and trav[r0, c0]:
        return rc
    rlo, rhi = max(0, r0 - max_cells), min(h, r0 + max_cells + 1)
    clo, chi = max(0, c0 - max_cells), min(w, c0 + max_cells + 1)
    cells = np.argwhere(trav[rlo:rhi, clo:chi])
    if len(cells) == 0:
        return None
    cells = cells + np.array([rlo, clo])
    d2 = (cells[:, 0] - r0) ** 2 + (cells[:, 1] - c0) ** 2
    best = cells[int(np.argmin(d2))]
    return int(best[0]), int(best[1])


def frontier_clusters(trav, unknown, dist, min_cells):
    """Reachable frontier cells (traversable, touching unknown), grouped into
    8-connected clusters. Each cluster is returned as a dict:
      goal  : (row, col) of the cluster cell closest to the robot (by BFS steps)
      steps : BFS steps from the robot to that cell
      size  : number of cells in the cluster
    """
    h, w = trav.shape
    frontier = trav & dilate(unknown, 1)
    reach = np.array(dist, dtype=np.int64).reshape(h, w) >= 0
    frontier &= reach
    fl = frontier.ravel().tolist()
    seen = [False] * (h * w)
    offs = (-w, w, -1, 1, -w - 1, -w + 1, w - 1, w + 1)
    clusters = []
    for start in np.flatnonzero(frontier).tolist():
        if seen[start]:
            continue
        seen[start] = True
        q = deque([start])
        members = []
        while q:
            c = q.popleft()
            members.append(c)
            for o in offs:
                n = c + o
                if 0 <= n < h * w and fl[n] and not seen[n]:
                    seen[n] = True
                    q.append(n)
        if len(members) < min_cells:
            continue
        best = min(members, key=lambda m: dist[m])
        clusters.append({'goal': divmod(best, w), 'steps': dist[best],
                         'size': len(members)})
    return clusters


def extract_path(parent, w, goal_rc):
    """Cells from the BFS start to goal_rc, as a list of (row, col)."""
    c = goal_rc[0] * w + goal_rc[1]
    path = []
    while c >= 0:
        path.append(divmod(c, w))
        c = parent[c]
    path.reverse()
    return path


def line_of_sight(trav, a, b):
    """True if the straight segment between cells a and b stays on traversable cells."""
    (r0, c0), (r1, c1) = a, b
    n = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
    for i in range(n + 1):
        t = i / n
        r = int(round(r0 + (r1 - r0) * t))
        c = int(round(c0 + (c1 - c0) * t))
        if not trav[r, c]:
            return False
    return True


def pick_carrot(trav, path, lookahead_cells):
    """Farthest path cell within lookahead (in cells, along the path) that the
    drone can reach in a straight line from the path start."""
    if not path:
        return None
    carrot = path[0]
    travelled = 0.0
    for i in range(1, len(path)):
        (r0, c0), (r1, c1) = path[i - 1], path[i]
        travelled += math.hypot(r1 - r0, c1 - c0)
        if travelled > lookahead_cells:
            break
        if line_of_sight(trav, path[0], path[i]):
            carrot = path[i]
        else:
            break
    return carrot
