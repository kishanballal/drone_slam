"""Offline test of the M3 logic on a synthetic copy of building.sdf (no ROS or Gazebo needed).

Run:  python3 -m pytest -q test/test_core_logic.py     (or: python3 test/test_core_logic.py)
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from slam_drone_m3.map3d_core import (VoxelMap, height_colors, pack_xyzrgb, quat_to_rot,  # noqa: E402
                                      filter_sensor_points, transform_points)
from slam_drone_m3.victim_core import detect, VictimTracker  # noqa: E402

RNG = np.random.default_rng(0)


def box_surface(cx, cy, cz, sx, sy, sz, step=0.04, yaw=0.0):
    """Points on the surface of a box (centre, size); bottom face skipped (sits on floor)."""
    xs = np.arange(-sx / 2, sx / 2 + 1e-9, step)
    ys = np.arange(-sy / 2, sy / 2 + 1e-9, step)
    zs = np.arange(-sz / 2, sz / 2 + 1e-9, step)
    faces = []
    X, Y = np.meshgrid(xs, ys)
    faces.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, sz / 2)])
    for sgn in (-1, 1):
        X, Z = np.meshgrid(xs, zs)
        faces.append(np.c_[X.ravel(), np.full(X.size, sgn * sy / 2), Z.ravel()])
        Y, Z = np.meshgrid(ys, zs)
        faces.append(np.c_[np.full(Y.size, sgn * sx / 2), Y.ravel(), Z.ravel()])
    p = np.vstack(faces)
    c, s = np.cos(yaw), np.sin(yaw)
    p = p @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
    return p + [cx, cy, cz]


def sphere_surface(cx, cy, cz, r, n=400):
    v = RNG.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v = v[v[:, 2] > -0.3]
    return v * r + [cx, cy, cz]


def victim_sitting(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    def at(dx, dy):
        return x + c * dx - s * dy, y + s * dx + c * dy
    parts = [box_surface(*at(0.22, 0), 0.08, 0.45, 0.36, 0.16, yaw=yaw),
             box_surface(*at(-0.10, 0), 0.41, 0.26, 0.38, 0.50, yaw=yaw),
             sphere_surface(*at(-0.08, 0), 0.79, 0.11)]
    return np.vstack(parts)


def victim_lying(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.vstack([box_surface(x, y, 0.11, 1.45, 0.42, 0.22, yaw=yaw),
                      sphere_surface(x + c * 0.84, y + s * 0.84, 0.12, 0.11)])


def building():
    w = []
    # outer walls 20 x 16, 3 m tall, 0.2 thick
    w += [box_surface(0, -8, 1.5, 20, 0.2, 3), box_surface(0, 8, 1.5, 20, 0.2, 3),
          box_surface(-10, 0, 1.5, 0.2, 16, 3), box_surface(10, 0, 1.5, 0.2, 16, 3)]
    w += [box_surface(0, 3, 1.5, 10, 3, 3)]                       # island
    w += [box_surface(-8.3, -1.5, 1.5, 3.4, 0.2, 3),              # partition with 2 doorways
          box_surface(-0.4, -1.5, 1.5, 10.0, 0.2, 3),
          box_surface(7.8, -1.5, 1.5, 4.4, 0.2, 3)]
    w += [box_surface(0, -4.75, 1.5, 0.2, 6.5, 3)]                # room divider
    w += [box_surface(7.5, 2, 1.5, 0.5, 0.5, 3), box_surface(-7.5, 5, 1.5, 0.5, 0.5, 3)]  # pillars
    furniture = [box_surface(-5, -5, 0.725, 1.2, 0.8, 0.05),      # table top + 4 legs
                 *[box_surface(-5 + dx, -5 + dy, 0.35, 0.05, 0.05, 0.7)
                   for dx in (-0.55, 0.55) for dy in (-0.35, 0.35)],
                 box_surface(6, -6.5, 0.9, 1.0, 0.4, 1.8),        # shelf
                 box_surface(3, 6.5, 0.5, 0.6, 0.6, 1.0)]         # crate (small = worst case)
    floor = np.c_[RNG.uniform(-10, 10, 40000), RNG.uniform(-8, 8, 40000), np.zeros(40000)]
    return np.vstack(w + furniture + [floor])


TRUTH = [('sitting', -8.0, -6.5), ('lying', 3.0, -4.5), ('sitting', -3.0, 7.2), ('lying', 8.6, 5.5)]


def world_points():
    v = [victim_sitting(-8.0, -6.5, 0.6), victim_lying(3.0, -4.5, 0.0),
         victim_sitting(-3.0, 7.2, -1.57), victim_lying(8.6, 5.5, 1.57)]
    return np.vstack([building()] + v)


def build_map():
    vm = VoxelMap(voxel=0.1)
    pts = world_points()
    for _ in range(3):  # three noisy "scans"
        vm.insert(pts + RNG.normal(0, 0.01, pts.shape))
    return vm


def test_map_and_colours():
    vm = build_map()
    p = vm.points(min_hits=2)
    assert len(p) > 10000
    rgb = height_colors(p[:, 2], 0.0, 3.0)
    assert rgb.shape == (len(p), 3)
    lo = height_colors(np.array([0.0]), 0, 3)[0]
    hi = height_colors(np.array([3.0]), 0, 3)[0]
    assert tuple(lo) == (0, 0, 255) and tuple(hi) == (255, 0, 0)
    assert len(pack_xyzrgb(p, rgb)) == 16 * len(p)


def test_transform_and_filter():
    R = quat_to_rot(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))  # yaw 90 deg
    p = transform_points(np.array([[1.0, 0, 0]]), R, [1, 2, 3])
    assert np.allclose(p, [[1, 3, 3]])
    raw = np.array([[0.1, 0, 0], [np.inf, 0, 0], [np.nan, 1, 1], [3, 0, 0], [20, 0, 0]])
    assert len(filter_sensor_points(raw, 0.6, 10.0)) == 1


def test_detects_all_victims_and_no_furniture():
    vm = build_map()
    victims, cands = detect(vm.points(min_hits=2))
    for c in cands:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in c.items()})
    assert len(victims) == len(TRUTH), victims
    for label, x, y in TRUTH:
        d = min(np.hypot(v['x'] - x, v['y'] - y) for v in victims)
        match = min(victims, key=lambda v: np.hypot(v['x'] - x, v['y'] - y))
        assert d < 0.4, (label, x, y, d)
        assert match['label'] == label, (label, match)


def test_tracker_confirms_after_three():
    tr = VictimTracker(min_hits=3)
    det = [{'x': 1.0, 'y': 1.0, 'label': 'lying'}]
    assert tr.update(det) == [] and tr.update(det) == []
    new = tr.update(det)
    assert len(new) == 1 and new[0]['id'] == 1
    tr.update([])  # confirmed victims persist
    assert len(tr.confirmed()) == 1


if __name__ == '__main__':
    test_map_and_colours()
    test_transform_and_filter()
    test_detects_all_victims_and_no_furniture()
    test_tracker_confirms_after_three()
    print('ALL TESTS PASSED')
