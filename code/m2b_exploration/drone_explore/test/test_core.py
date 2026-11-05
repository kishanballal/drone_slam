"""Unit tests for drone_explore.core (no ROS needed):  python3 -m pytest test/"""
import math

import numpy as np

from drone_explore.core import (body_to_ned, world_delta_to_body, traversable_mask, bfs,
                                frontier_clusters, extract_path, pick_carrot, wrap)


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_body_to_ned_facing_north():
    assert close(body_to_ned(1.0, 0.0, 0.0), (1.0, 0.0))     # forward = north
    assert close(body_to_ned(0.0, 1.0, 0.0), (0.0, -1.0))    # left = west


def test_body_to_ned_facing_east():
    h = math.pi / 2
    assert close(body_to_ned(1.0, 0.0, h), (0.0, 1.0))       # forward = east
    assert close(body_to_ned(0.0, 1.0, h), (1.0, 0.0))       # left = north


def test_enu_world_to_body_to_ned_roundtrip():
    # Drone at ENU yaw psi has PX4 heading pi/2 - psi. A world ENU offset must
    # come out as the same physical offset in NED (north = dy, east = dx).
    for psi in (0.0, 0.7, -2.0, 3.0):
        heading = wrap(math.pi / 2 - psi)
        dx, dy = 1.3, -0.4
        fwd, left = world_delta_to_body(dx, dy, psi)
        assert close(body_to_ned(fwd, left, heading), (dy, dx), 1e-9)


def two_rooms():
    """20x10 cells: left half known free, right half unknown, wall border."""
    g = np.full((10, 20), -1, dtype=np.int16)
    g[:, :10] = 0
    g[0, :] = g[-1, :] = 100
    g[:, 0] = 100
    return g


def test_frontier_found_at_known_unknown_boundary():
    trav, unknown = traversable_mask(two_rooms(), 1)
    dist, parent = bfs(trav, (5, 3))
    clusters = frontier_clusters(trav, unknown, dist, 2)
    assert len(clusters) == 1
    goal = clusters[0]['goal']
    assert goal[1] == 9                       # last free column before unknown
    path = extract_path(parent, trav.shape[1], goal)
    assert path[0] == (5, 3) and path[-1] == goal
    carrot = pick_carrot(trav, path, 3.0)
    assert math.hypot(carrot[0] - 5, carrot[1] - 3) <= 3.0 + 1e-9


def test_no_frontier_when_fully_known():
    g = np.zeros((10, 10), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    trav, unknown = traversable_mask(g, 1)
    dist, _ = bfs(trav, (5, 5))
    assert frontier_clusters(trav, unknown, dist, 1) == []
