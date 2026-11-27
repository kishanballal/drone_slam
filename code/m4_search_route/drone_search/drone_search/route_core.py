"""M4 search-route planner core (pure numpy, no ROS, so it can be unit-tested offline).

Input : the 2D occupancy grid the drone built while exploring (+ where it already flew).
Output: a short list of search stops and the shortest order to visit them.

Steps
 1. Coarsen the map to `cell` metres and work out each cell's distance to the nearest wall.
 2. Split the free space into areas (rooms / corridors): shrink free space by `door_split`
    so doorways close, take the connected blobs as room cores, then grow them back.
 3. Coverage: a stop "sees" every free cell within `see_radius` that has a clear line of sight.
    Cells already seen from the exploration flight path are counted as done.
    Greedy set cover picks the fewest stops that see (almost) everything that is left.
 4. Real flying distances between all stops (Dijkstra on cells with enough wall clearance).
 5. Shortest order start -> every stop once -> home (exact Held-Karp up to 12 stops, else 2-opt).
"""
import heapq
import math
from collections import deque

import numpy as np

FREE, OCC, UNK = 0, 1, 2
N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class Grid:
    """Coarse planning grid. Row 0 = lowest y (same as nav_msgs/OccupancyGrid)."""

    def __init__(self, data, width, height, resolution, origin_xy, cell=0.2, occ_thresh=65):
        a = np.asarray(data, dtype=np.int16).reshape(height, width)
        f = max(1, int(round(cell / resolution)))
        h, w = height // f, width // f
        a = a[:h * f, :w * f].reshape(h, f, w, f).transpose(0, 2, 1, 3).reshape(h, w, f * f)
        occ = (a >= occ_thresh).any(axis=2)
        free_frac = ((a >= 0) & (a < occ_thresh)).mean(axis=2)
        self.cls = np.full((h, w), UNK, np.uint8)
        self.cls[free_frac >= 0.5] = FREE
        self.cls[occ] = OCC
        self.cell = resolution * f
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = h, w
        self.clear = self._clearance()

    # --- coordinates -------------------------------------------------------
    def to_cell(self, x, y):
        return int((y - self.oy) / self.cell), int((x - self.ox) / self.cell)

    def to_xy(self, r, c):
        return self.ox + (c + 0.5) * self.cell, self.oy + (r + 0.5) * self.cell

    def inside(self, r, c):
        return 0 <= r < self.h and 0 <= c < self.w

    # --- distance to nearest wall / unknown (exact, chunked brute force) ----
    def _clearance(self):
        free = np.argwhere(self.cls == FREE)
        obst = np.argwhere(self.cls != FREE)
        d = np.zeros((self.h, self.w))
        if len(free) == 0:
            return d
        if len(obst) == 0:
            d[self.cls == FREE] = 1e9
            return d
        best = np.empty(len(free))
        for i in range(0, len(free), 512):
            p = free[i:i + 512, None, :].astype(np.float32)
            dd = ((p - obst[None, :, :]) ** 2).sum(axis=2)
            best[i:i + 512] = np.sqrt(dd.min(axis=1))
        d[free[:, 0], free[:, 1]] = best * self.cell
        return d

    def snap(self, x, y, min_clear):
        """Nearest cell to (x, y) with at least `min_clear` m of clearance."""
        ok = np.argwhere(self.clear >= min_clear)
        if len(ok) == 0:
            return None
        r, c = self.to_cell(x, y)
        i = int(((ok - np.array([r, c])) ** 2).sum(axis=1).argmin())
        return int(ok[i, 0]), int(ok[i, 1])


# --- 2. rooms ---------------------------------------------------------------
def segment_areas(g, door_split=1.0, min_area=2.0):
    """Label every free cell with an area id 1..n (0 = not free)."""
    core = g.clear >= door_split
    lab = np.zeros((g.h, g.w), np.int32)
    n = 0
    for r, c in np.argwhere(core):
        if lab[r, c]:
            continue
        n += 1
        comp, q = [(r, c)], deque([(r, c)])
        lab[r, c] = n
        while q:
            a, b = q.popleft()
            for da, db in N8:
                y, x = a + da, b + db
                if g.inside(y, x) and core[y, x] and not lab[y, x]:
                    lab[y, x] = n
                    comp.append((y, x))
                    q.append((y, x))
        if len(comp) * g.cell ** 2 < min_area:       # tiny core: let a neighbour absorb it
            for y, x in comp:
                lab[y, x] = -1
            n -= 1
    lab[lab < 0] = 0
    # grow the cores back over all free cells (closest core wins)
    q = deque(map(tuple, np.argwhere(lab > 0)))
    while q:
        a, b = q.popleft()
        for da, db in N8:
            y, x = a + da, b + db
            if g.inside(y, x) and g.cls[y, x] == FREE and not lab[y, x]:
                lab[y, x] = lab[a, b]
                q.append((y, x))
    return lab, n


# --- 3. coverage --------------------------------------------------------------
class Visibility:
    def __init__(self, g, see_radius=4.5, target_step=2):
        self.g = g
        self.R = see_radius
        self.blocked = g.cls == OCC
        t = np.argwhere(g.cls == FREE)
        self.targets = t[(t[:, 0] % target_step == 0) & (t[:, 1] % target_step == 0)]
        self.txy = (self.targets + 0.5) * g.cell          # (row, col) in metres

    def seen_from(self, r, c):
        """Boolean mask over self.targets: visible from cell (r, c)."""
        g = self.g
        p = (np.array([r, c]) + 0.5) * g.cell
        d = np.sqrt(((self.txy - p) ** 2).sum(axis=1))
        near = np.nonzero(d <= self.R)[0]
        out = np.zeros(len(self.targets), bool)
        if len(near) == 0:
            return out
        steps = max(2, int(math.ceil(self.R / (0.5 * g.cell))))
        s = np.linspace(0.0, 1.0, steps)[None, :, None]
        pts = p[None, None, :] + s * (self.txy[near][:, None, :] - p[None, None, :])
        cells = np.floor(pts / g.cell).astype(int)
        cells[..., 0] = cells[..., 0].clip(0, g.h - 1)
        cells[..., 1] = cells[..., 1].clip(0, g.w - 1)
        hit = self.blocked[cells[..., 0], cells[..., 1]].any(axis=1)
        out[near[~hit]] = True
        return out


def choose_stops(g, vis, flown_cells=(), stop_clear=0.8, cover_goal=0.95,
                 min_gain_m2=1.0, max_stops=12, cand_step=2, start=None, reach_m=None):
    """Greedy set cover. Returns (stops [(r,c)], info dict).
    reach_m: if set, a candidate's gain is discounted by exp(-d / reach_m), d = straight-line
    distance to the start or the nearest stop already chosen, so stops stay close together."""
    seen = np.zeros(len(vis.targets), bool)
    for r, c in flown_cells:
        if g.inside(r, c):
            seen |= vis.seen_from(r, c)
    cand = np.argwhere(g.clear >= stop_clear)
    cand = cand[(cand[:, 0] % cand_step == 0) & (cand[:, 1] % cand_step == 0)]
    cover = [vis.seen_from(r, c) for r, c in cand]
    reachable = np.zeros(len(vis.targets), bool)
    for m in cover:
        reachable |= m
    todo = reachable & ~seen
    total = int(todo.sum())
    per_target_m2 = (g.cell * 2) ** 2
    min_gain = max(1, int(min_gain_m2 / per_target_m2))
    ks = []                                   # chosen candidate indices
    anchors = [start] if start is not None else []
    while len(ks) < max_stops and total and (1.0 - todo.sum() / total) < cover_goal:
        gains = np.array([(m & todo).sum() for m in cover], float)
        score = gains.copy()
        if reach_m and anchors:
            a = np.array(anchors, float)
            d = np.sqrt(((cand[:, None, :] - a[None]) ** 2).sum(axis=2)).min(axis=1) * g.cell
            score *= np.exp(-d / reach_m)
        score[gains < min_gain] = -1
        k = int(score.argmax())
        if score[k] < 0:
            break
        ks.append(k)
        anchors.append(tuple(cand[k]))
        todo &= ~cover[k]

    need = int(np.ceil(cover_goal * total))   # targets that must be covered (of reachable & ~seen)
    base = reachable & ~seen

    def covered(idx):
        m = np.zeros(len(vis.targets), bool)
        for i in idx:
            m |= cover[i]
        return int((m & base).sum())

    # drop stops that became redundant
    for k in list(ks):
        rest = [x for x in ks if x != k]
        if covered(rest) >= need:
            ks = rest
    # merge: replace two nearby stops by one candidate if coverage still holds
    merged = True
    while merged and len(ks) > 1:
        merged = False
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                if np.hypot(*(cand[ks[i]] - cand[ks[j]])) * g.cell > 4.0:
                    continue
                rest = [x for n_, x in enumerate(ks) if n_ not in (i, j)]
                m = np.zeros(len(vis.targets), bool)
                for x in rest:
                    m |= cover[x]
                have = int((m & base).sum())
                gain = np.array([(c & base & ~m).sum() for c in cover])
                k = int(gain.argmax())
                if have + gain[k] >= need:
                    ks = rest + [k]
                    merged = True
                    break
            if merged:
                break
    stops = [(int(cand[k, 0]), int(cand[k, 1])) for k in ks]
    m = seen.copy()
    for k in ks:
        m |= cover[k]
    todo = reachable & ~m
    info = {
        'targets': len(vis.targets),
        'seen_while_exploring': float(seen.mean()) if len(seen) else 0.0,
        'left_after_exploring_m2': total * per_target_m2,
        'covered_fraction': 1.0 - (todo.sum() / total) if total else 1.0,
    }
    return stops, info


# --- 4. distances -----------------------------------------------------------
def dijkstra(g, src, travel_clear=0.5):
    """Cost-to-go (m) and parents from src over cells with clearance >= travel_clear."""
    ok = g.clear >= travel_clear
    ok[src] = True
    dist = np.full((g.h, g.w), np.inf)
    par = -np.ones((g.h, g.w, 2), np.int32)
    dist[src] = 0.0
    pq = [(0.0, src)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        for dr, dc in N8:
            y, x = r + dr, c + dc
            if not (g.inside(y, x) and ok[y, x]):
                continue
            nd = d + (g.cell * (1.41421356 if dr and dc else 1.0))
            if nd < dist[y, x]:
                dist[y, x] = nd
                par[y, x] = (r, c)
                heapq.heappush(pq, (nd, (y, x)))
    return dist, par


def trace(par, src, dst):
    path, cur = [dst], dst
    while cur != src:
        p = tuple(int(v) for v in par[cur])
        if p[0] < 0:
            return []
        path.append(p)
        cur = p
    return path[::-1]


def distance_matrix(g, nodes, travel_clear=0.5):
    n = len(nodes)
    D = np.zeros((n, n))
    pars = []
    for i, s in enumerate(nodes):
        dist, par = dijkstra(g, s, travel_clear)
        pars.append(par)
        for j, t in enumerate(nodes):
            D[i, j] = dist[t]
    return D, pars


# --- 5. order -----------------------------------------------------------------
def best_order(D, start=0, end=None):
    """Shortest open tour start -> all other nodes -> end. Returns (order, length).
    `end` (index) is visited last; None = finish at the last stop."""
    n = len(D)
    mid = [i for i in range(n) if i != start and i != end]
    if not mid:
        order = [start] + ([end] if end is not None else [])
        return order, tour_length(D, order)
    if len(mid) <= 12:
        order = _held_karp(D, start, end, mid)
    else:
        order = _two_opt(D, start, end, mid)
    return order, tour_length(D, order)


def tour_length(D, order):
    return float(sum(D[a, b] for a, b in zip(order, order[1:])))


def _held_karp(D, start, end, mid):
    m = len(mid)
    INF = float('inf')
    best = {}
    for k in range(m):
        best[(1 << k, k)] = (D[start, mid[k]], -1)
    for mask in range(1, 1 << m):
        for k in range(m):
            if not (mask >> k) & 1 or (mask, k) not in best:
                continue
            cost = best[(mask, k)][0]
            for j in range(m):
                if (mask >> j) & 1:
                    continue
                key = (mask | (1 << j), j)
                c = cost + D[mid[k], mid[j]]
                if c < best.get(key, (INF,))[0]:
                    best[key] = (c, k)
    full = (1 << m) - 1
    last = min(range(m), key=lambda k: best[(full, k)][0] + (D[mid[k], end] if end is not None else 0.0))
    seq, mask, k = [], full, last
    while k != -1:
        seq.append(mid[k])
        k, mask = best[(mask, k)][1], mask & ~(1 << k)
    return [start] + seq[::-1] + ([end] if end is not None else [])


def _two_opt(D, start, end, mid):
    left, seq, cur = set(mid), [], start          # nearest neighbour first
    while left:
        nxt = min(left, key=lambda j: D[cur, j])
        seq.append(nxt)
        left.remove(nxt)
        cur = nxt
    order = [start] + seq + ([end] if end is not None else [])
    last = len(order) - (1 if end is not None else 0)
    improved = True
    while improved:
        improved = False
        for i in range(1, last - 1):
            for k in range(i + 1, last):
                new = order[:i] + order[i:k + 1][::-1] + order[k + 1:]
                if tour_length(D, new) < tour_length(D, order) - 1e-6:
                    order, improved = new, True
    return order


# --- all together --------------------------------------------------------------
def plan_route(g, start_xy, home_xy=(0.0, 0.0), flown_xy=(), see_radius=5.0,
               stop_clear=0.8, travel_clear=0.5, door_split=1.0, cover_goal=0.95,
               min_gain_m2=1.0, max_stops=12, reach_options=(0.0, 12.0, 6.0)):
    """Returns a dict with stops (x, y, area) in visiting order, path, lengths and stats.
    Tries the cover with a few `reach_options` (0 = ignore distance) and keeps the shortest tour."""
    lab, n_areas = segment_areas(g, door_split)
    vis = Visibility(g, see_radius)
    flown = [g.to_cell(x, y) for x, y in flown_xy]
    s = g.snap(*start_xy, travel_clear) or g.to_cell(*start_xy)
    e = g.snap(*home_xy, travel_clear) or g.to_cell(*home_xy)
    best = None
    for reach_m in reach_options:
        stops, info = choose_stops(g, vis, flown, stop_clear, cover_goal, min_gain_m2,
                                   max_stops, start=s, reach_m=reach_m or None)
        nodes = [s, e] + stops
        D, pars = distance_matrix(g, nodes, travel_clear)
        reach = [i for i in range(2, len(nodes)) if np.isfinite(D[0, i])]
        sub = [0, 1] + reach
        order_sub, length = best_order(D[np.ix_(sub, sub)], start=0, end=1)
        if best is None or length < best[0]:
            best = (length, nodes, pars, [sub[i] for i in order_sub], stops, reach, info)
    length, nodes, pars, order, stops, reach, info = best
    path = []
    for a, b in zip(order, order[1:]):
        seg = trace(pars[a], nodes[a], nodes[b])
        path += seg if not path else seg[1:]
    out_stops = []
    for i in order[1:-1]:
        r, c = nodes[i]
        x, y = g.to_xy(r, c)
        out_stops.append((x, y, int(lab[r, c])))
    return {
        'stops': out_stops,
        'home': g.to_xy(*e),
        'length': length,
        'path_xy': [g.to_xy(r, c) for r, c in path],
        'n_areas': n_areas,
        'areas_m2': {k: float((lab == k).sum() * g.cell ** 2) for k in range(1, n_areas + 1)},
        'unreachable_stops': len(stops) - len(reach),
        'labels': lab,
        **info,
    }


def fixed_tour_length(g, start_xy, stops_xy, travel_clear=0.5):
    """Flying distance of a hand-written stop list (for the before/after comparison)."""
    nodes = [g.snap(*start_xy, travel_clear)] + [g.snap(x, y, travel_clear) for x, y in stops_xy]
    D, _ = distance_matrix(g, nodes, travel_clear)
    return tour_length(D, list(range(len(nodes))))
