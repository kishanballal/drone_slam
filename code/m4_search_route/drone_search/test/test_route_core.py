"""Offline tests for the M4 route planner (no ROS needed): python3 -m pytest -q test"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_search import route_core as rc  # noqa: E402

HERE = os.path.dirname(__file__)
M3_TOUR = [(-6, -4.5), (-2.5, -5), (-8.5, 2), (-5, 6), (1.5, 6), (6.5, 5.5), (6, -4), (1.5, -3), (0, 0)]


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        a = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)[::-1]
    occ = np.where(a >= 250, 0, np.where(a <= 50, 100, -1))
    return occ.ravel(), w, h


def m3_grid():
    data, w, h = load_pgm(os.path.join(HERE, 'm3_building.pgm'))
    return rc.Grid(data, w, h, 0.05, (-9.959, -7.915), cell=0.2)


def two_rooms():
    """10 x 5 m box split by a wall at x = 5 with a 1.8 m door."""
    res, w, h = 0.05, 200, 100
    a = np.zeros((h, w), int)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    a[:, 100] = 100
    a[41:77, 100] = 0
    return rc.Grid(a.ravel(), w, h, res, (0.0, 0.0), cell=0.2)


def test_two_rooms_split_and_one_stop_each():
    g = two_rooms()
    lab, n = rc.segment_areas(g, 1.0)
    assert n == 2
    res = rc.plan_route(g, (1.0, 2.5), (1.0, 2.5), see_radius=3.0)
    areas = {a for _, _, a in res['stops']}
    assert areas == {1, 2}
    assert res['unreachable_stops'] == 0


def test_held_karp_matches_brute_force():
    import itertools
    rng = np.random.default_rng(1)
    P = rng.uniform(0, 10, (7, 2))
    D = np.sqrt(((P[:, None] - P[None]) ** 2).sum(axis=2))
    order, L = rc.best_order(D, 0, 1)
    best = min(rc.tour_length(D, [0, *p, 1]) for p in itertools.permutations(range(2, 7)))
    assert abs(L - best) < 1e-9 and order[0] == 0 and order[-1] == 1


def test_two_opt_close_to_exact():
    rng = np.random.default_rng(2)
    P = rng.uniform(0, 10, (10, 2))
    D = np.sqrt(((P[:, None] - P[None]) ** 2).sum(axis=2))
    exact = rc.tour_length(D, rc._held_karp(D, 0, 1, list(range(2, 10))))
    approx = rc.tour_length(D, rc._two_opt(D, 0, 1, list(range(2, 10))))
    assert approx <= exact * 1.15


def test_m3_building_three_areas_and_shorter_than_fixed_tour():
    g = m3_grid()
    lab, n = rc.segment_areas(g, 1.0)
    assert n == 3                      # south-west room, south-east room, north loop
    flown = [(x, 0.0) for x in np.arange(-8, 8, 0.5)]   # exploration flew the corridor
    res = rc.plan_route(g, (0.0, 0.0), (0.0, 0.0), flown)
    fixed = rc.fixed_tour_length(g, (0.0, 0.0), M3_TOUR)
    assert len(res['stops']) < len(M3_TOUR)
    assert res['length'] < fixed
    assert res['covered_fraction'] >= 0.95
    assert {a for _, _, a in res['stops']} == {1, 2, 3}   # every area visited
    for x, y, _ in res['stops']:
        r, c = g.to_cell(x, y)
        assert g.clear[r, c] >= 0.8


def test_route_path_stays_off_walls():
    g = m3_grid()
    res = rc.plan_route(g, (0.0, 0.0), (0.0, 0.0))
    for x, y in res['path_xy']:
        r, c = g.to_cell(x, y)
        assert g.cls[r, c] == rc.FREE
