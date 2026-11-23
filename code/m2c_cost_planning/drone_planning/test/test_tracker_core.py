import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_planning.tracker_core import StaticMap, Tracker, cluster, scan_points  # noqa: E402


def room():
    """10 x 10 m room, 0.05 m cells, walls around the edge."""
    g = np.zeros((200, 200), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    return StaticMap(g, 0.05, 0.0, 0.0)


def fake_scan(obj=None, pose=(5.0, 5.0, 0.0), n=360):
    """Ray-cast a square room (+ an optional round object (x, y, r)) from pose."""
    px, py, _ = pose
    ranges = []
    for i in range(n):
        a = -math.pi + i * 2 * math.pi / n
        dx, dy = math.cos(a), math.sin(a)
        ts = [t for t in ((0.05 - px) / dx if dx < 0 else (9.95 - px) / dx if dx else 1e9,
                          (0.05 - py) / dy if dy < 0 else (9.95 - py) / dy if dy else 1e9)]
        r = min(t for t in ts if t > 0)
        if obj:
            ox, oy, rad = obj
            fx, fy = px - ox, py - oy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - rad * rad
            disc = b * b - c
            if disc >= 0 and -b - math.sqrt(disc) > 0:
                r = min(r, -b - math.sqrt(disc))
        ranges.append(r)
    return ranges, -math.pi, 2 * math.pi / n


def detect(m, ranges, a0, inc, pose=(5.0, 5.0, 0.0)):
    pts = [p for p in scan_points(ranges, a0, inc, pose, rmax=12.0) if m.is_dynamic_candidate(*p)]
    return cluster(pts)


def test_walls_are_not_dynamic():
    m = room()
    assert detect(m, *fake_scan()) == []


def test_object_is_detected_where_it_is():
    m = room()
    dets = detect(m, *fake_scan((7.0, 6.0, 0.25)))
    assert len(dets) == 1
    assert math.hypot(dets[0][0] - 7.0, dets[0][1] - 6.0) < 0.3   # front surface of the blob


def test_tracker_gives_id_and_speed():
    m, tr = room(), Tracker()
    first = []
    for k in range(20):                       # object walks +x at 0.4 m/s, scans at 10 Hz
        t = k * 0.1
        new, _ = tr.update(detect(m, *fake_scan((2.0 + 0.4 * t, 7.0, 0.25))), t)
        first += new
    assert first == [1]
    track = tr.confirmed()[1]
    assert abs(Tracker.speed(track) - 0.4) < 0.1
    assert track['vx'] > 0.3


def test_track_dropped_when_object_gone():
    m, tr = room(), Tracker(timeout=0.5)
    for k in range(5):
        tr.update(detect(m, *fake_scan((3.0, 3.0, 0.25))), k * 0.1)
    _, dropped = tr.update([], 2.0)
    assert dropped == [1] and not tr.confirmed()
