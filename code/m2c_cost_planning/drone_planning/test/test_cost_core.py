"""Offline tests for the cost-aware planner (no ROS needed).

  python3 -m pytest -q test
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_planning.cost_core import (CostParams, LETHAL, downsample, evaluate,  # noqa: E402
                                      path_blocked, paths_differ, plan)

RES = 0.1


def inflate(occ, radius_m=0.6, inscribed_m=0.4, res=RES):
    """Rough copy of Nav2's inflation layer on a 0/100 grid."""
    occd = occ >= LETHAL
    h, w = occ.shape
    n = int(radius_m / res) + 1
    d = np.full(occ.shape, np.inf)
    d[occd] = 0.0
    for dy in range(-n, n + 1):          # brute-force distance to the nearest obstacle
        for dx in range(-n, n + 1):
            r = math.hypot(dx, dy) * res
            if r > radius_m:
                continue
            sh = np.zeros_like(occd)
            sh[max(0, dy):h + min(0, dy), max(0, dx):w + min(0, dx)] = \
                occd[max(0, -dy):h + min(0, -dy), max(0, -dx):w + min(0, -dx)]
            d[sh] = np.minimum(d[sh], r)
    g = np.zeros_like(occ, dtype=np.int16)
    infl = (d > 0) & (d <= radius_m)
    g[infl] = (1 + 97 * np.exp(-4.0 * (d[infl] - inscribed_m)).clip(0, 1)).astype(np.int16)
    g[(d > 0) & (d <= inscribed_m)] = 99
    g[occ >= LETHAL] = 100
    return g


def loop_world():
    """20 m x 12 m box with a solid block inside: a 2 m wide ring corridor (like the building loop)."""
    w, h = int(20 / RES), int(12 / RES)
    occ = np.zeros((h, w), dtype=np.int16)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = LETHAL
    occ[int(2 / RES):int(10 / RES), int(2 / RES):int(18 / RES)] = LETHAL   # inner block
    return occ


def cell(x, y):
    return int(x / RES), int(y / RES)


def test_plans_around_loop_and_is_fast():
    g = inflate(loop_world())
    p = CostParams()
    t = time.time()
    r = plan(g, RES, cell(1, 1), cell(19, 11), p, soc=1.0)
    dt = time.time() - t
    assert r.error == '', r.error
    assert r.terms['D_m'] > 25.0
    assert r.terms['turns'] <= 4          # complexity term keeps it to ~1-2 turns
    assert dt < 1.5, dt


def test_replans_when_corridor_blocked():
    occ = loop_world()
    g = inflate(occ)
    p = CostParams()
    a = plan(g, RES, cell(1, 6), cell(19, 6), p)
    assert a.error == ''
    went_south = a.cells[len(a.cells) // 2][1] < int(6 / RES)
    # drop a 0.5 m "walker" across the corridor the first path used
    y0 = 0.5 if went_south else 10.5
    occ2 = occ.copy()
    occ2[int((y0 - 0.5) / RES):int((y0 + 1.5) / RES), int(9.75 / RES):int(10.25 / RES)] = LETHAL
    g2 = inflate(occ2)
    assert path_blocked(g2, a.cells) is not None
    b = plan(g2, RES, cell(1, 6), cell(19, 6), p)
    assert b.error == ''
    assert path_blocked(g2, b.cells) is None
    assert paths_differ(a.cells, b.cells, RES, horizon_m=15.0)


def test_risk_weight_keeps_drone_off_walls():
    g = inflate(loop_world())
    lo = plan(g, RES, cell(1, 1), cell(19, 1), CostParams(w_risk=0.0))
    hi = plan(g, RES, cell(1, 1), cell(19, 1), CostParams(w_risk=8.0))
    assert hi.terms['R_m'] <= lo.terms['R_m'] + 1e-9


def test_low_battery_weights_energy_more():
    p = CostParams()
    g = inflate(loop_world())
    full = plan(g, RES, cell(1, 1), cell(19, 11), p, soc=1.0)
    low = plan(g, RES, cell(1, 1), cell(19, 11), p, soc=0.2)
    assert low.terms['w_battery_eff'] > full.terms['w_battery_eff']
    assert low.terms['E_Wh'] <= full.terms['E_Wh'] + 1e-9


def test_evaluate_counts_turns():
    g = np.zeros((10, 10), dtype=np.int16)
    cells = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    t = evaluate(g, 1.0, cells, CostParams(), start_yaw=0.0)
    assert t['turns'] == 1
    assert math.isclose(t['K_rad'], math.pi / 2 + CostParams().turn_count_cost)


def test_downsample_keeps_obstacles():
    g = np.full((4, 4), -1, dtype=np.int16)
    g[0, 0] = 100
    d = downsample(g, 2)
    assert d[0, 0] == 100 and d[1, 1] == -1


def test_doorway_keeps_wall_margin():
    """A 1.8 m doorway: with block_at=45 the path keeps >= 0.7 m from the door frame."""
    res, H, W = 0.1, 80, 100
    occ = np.zeros((H, W), bool)
    occ[40, :] = True
    occ[40, 46:64] = False
    ys, xs = np.nonzero(occ)
    Y, X = np.mgrid[0:H, 0:W]
    d = np.full((H, W), 1e9)
    for y, x in zip(ys, xs):
        d = np.minimum(d, np.hypot(Y - y, X - x) * res)
    grid = np.where(d <= 0.4, 99, np.where(d <= 0.7, np.round(99 * np.exp(-2.5 * (d - 0.4))), 0))
    grid = grid.astype(np.int16)
    grid[occ] = 100
    tight = plan(grid, res, (20, 20), (80, 60), CostParams(), block_at=99)
    safe = plan(grid, res, (20, 20), (80, 60), CostParams(), block_at=45)
    assert not tight.error and not safe.error
    m = min(d[y, x] for x, y in safe.cells)
    assert m >= 0.69
    assert m >= min(d[y, x] for x, y in tight.cells)
