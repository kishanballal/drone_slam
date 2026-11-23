"""Dynamic obstacle detection + tracking from a 2D LiDAR scan (no ROS imports, unit tested).

Idea: the SLAM map holds the walls. A scan point that lands on free, mapped space
far from any wall is something that is not part of the building -> a candidate.
Candidates are clustered (one cluster = one object), and clusters are tracked
from scan to scan (nearest neighbour), which gives each object an id and a velocity.
"""
import math

import numpy as np


def dilate(mask, r):
    """Binary dilation of a 2D bool mask by a square of radius r cells (numpy only)."""
    if r <= 0:
        return mask.copy()
    h, w = mask.shape
    pad = np.pad(mask, r)
    out = np.zeros_like(mask)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            out |= pad[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


class StaticMap:
    """Frozen copy of the SLAM map used as 'what is static'."""

    def __init__(self, grid, res, ox, oy, wall_margin=0.35, occ_thresh=65):
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        g = np.asarray(grid, dtype=np.int16)
        self.h, self.w = g.shape
        self.near_wall = dilate(g >= occ_thresh, int(math.ceil(wall_margin / self.res)))
        self.unknown = g < 0

    def is_dynamic_candidate(self, x, y):
        c, r = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        if not (0 <= c < self.w and 0 <= r < self.h):
            return False                      # outside the mapped area: can't tell
        return not (self.near_wall[r, c] or self.unknown[r, c])


def scan_points(ranges, angle_min, angle_inc, pose, rmin=0.2, rmax=8.0):
    """Scan -> list of (x, y) in the map frame. pose = (x, y, yaw) of the scanner in the map."""
    px, py, yaw = pose
    pts = []
    for i, r in enumerate(ranges):
        if not (rmin < r < rmax) or math.isnan(r):
            continue
        a = yaw + angle_min + i * angle_inc
        pts.append((px + r * math.cos(a), py + r * math.sin(a)))
    return pts


def cluster(points, gap=0.3, min_points=3, max_size=1.5):
    """Greedy Euclidean clustering. Returns [(cx, cy, n), ...] for blobs of a person/box size."""
    out, used = [], [False] * len(points)
    for i in range(len(points)):
        if used[i]:
            continue
        used[i] = True
        members, stack = [points[i]], [i]
        while stack:
            k = stack.pop()
            for j in range(len(points)):
                if not used[j] and math.dist(points[k], points[j]) < gap:
                    used[j] = True
                    stack.append(j)
                    members.append(points[j])
        if len(members) < min_points:
            continue
        xs, ys = [p[0] for p in members], [p[1] for p in members]
        if max(max(xs) - min(xs), max(ys) - min(ys)) > max_size:
            continue                          # too big: a missing wall, not an object
        out.append((sum(xs) / len(xs), sum(ys) / len(ys), len(members)))
    return out


class Tracker:
    """Nearest-neighbour tracker. A track is confirmed after `confirm` hits in a row."""

    def __init__(self, gate=1.0, confirm=3, timeout=1.5, alpha=0.4):
        self.gate, self.confirm, self.timeout, self.alpha = gate, confirm, timeout, alpha
        self.tracks = {}            # id -> dict(x, y, vx, vy, t, hits, confirmed)
        self.next_id = 1

    def update(self, detections, t):
        """detections: [(x, y, ...)]. Returns (new_confirmed_ids, dropped_ids)."""
        new, dropped = [], []
        free = set(self.tracks)
        for d in detections:
            x, y = d[0], d[1]
            best, bd = None, self.gate
            for tid in free:
                tr = self.tracks[tid]
                dt = t - tr['t']
                px, py = tr['x'] + tr['vx'] * dt, tr['y'] + tr['vy'] * dt
                dist = math.hypot(x - px, y - py)
                if dist < bd:
                    best, bd = tid, dist
            if best is None:
                self.tracks[self.next_id] = dict(x=x, y=y, vx=0.0, vy=0.0, t=t, hits=1,
                                                 confirmed=False)
                self.next_id += 1
                continue
            free.discard(best)
            tr = self.tracks[best]
            dt = max(1e-3, t - tr['t'])
            a = self.alpha
            tr['vx'] = (1 - a) * tr['vx'] + a * (x - tr['x']) / dt
            tr['vy'] = (1 - a) * tr['vy'] + a * (y - tr['y']) / dt
            tr['x'], tr['y'], tr['t'] = x, y, t
            tr['hits'] += 1
            if not tr['confirmed'] and tr['hits'] >= self.confirm:
                tr['confirmed'] = True
                new.append(best)
        for tid in list(self.tracks):
            if t - self.tracks[tid]['t'] > self.timeout:
                if self.tracks[tid]['confirmed']:
                    dropped.append(tid)
                del self.tracks[tid]
        return new, dropped

    def confirmed(self):
        return {k: v for k, v in self.tracks.items() if v['confirmed']}

    @staticmethod
    def speed(tr):
        return math.hypot(tr['vx'], tr['vy'])
