"""Pure-numpy victim detection on the accumulated 3D voxel map (no ROS imports, unit testable).

Idea: look at the map from above as columns of 0.1 m cells.
  * A column whose top is higher than max_object_height is building structure (wall, pillar, shelf).
  * The remaining low columns are grouped into clusters (8-connected).
  * Clusters touching structure are dropped (wall-base noise, things against walls).
  * Each cluster is measured (height, long side, short side via PCA, top flatness) and classified
    as 'lying' or 'sitting' person, or rejected (table, crate, noise).
  * A tracker only confirms a victim after it is seen in min_hits consecutive map updates.
"""
from collections import deque
import numpy as np

DEFAULT_PARAMS = {
    'cell': 0.1,
    'floor_cut': 0.08,            # ignore points below this (floor)
    'z_consider_max': 2.6,        # ignore points above this (roof, if the world has one)
    'max_object_height': 1.3,     # columns with a top above this are structure
    'min_cells': 6,
    # lying person: low and long
    'lying_h': (0.15, 0.5), 'lying_len': (1.0, 2.2), 'lying_wid': (0.2, 0.9),
    # sitting / slumped person: knee-to-head high, compact, NOT flat on top (has a head)
    'sitting_h': (0.6, 1.2), 'sitting_len': (0.35, 1.1), 'sitting_wid': (0.2, 0.9),
    'top_band': 0.12,             # cells within this of the max height count as "top"
    'max_top_fraction': 0.45,     # crates and tables are flat-topped (fraction ~1.0)
}

NEIGH8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def column_tops(pts, cell, floor_cut, z_max):
    """Return dict (ix, iy) -> highest z in that column, ignoring floor and roof."""
    p = pts[(pts[:, 2] > floor_cut) & (pts[:, 2] <= z_max)]
    if len(p) == 0:
        return {}
    ij = np.floor(p[:, :2] / cell).astype(np.int64)
    order = np.lexsort((p[:, 2], ij[:, 1], ij[:, 0]))
    ij, z = ij[order], p[order, 2]
    last = np.ones(len(ij), dtype=bool)
    last[:-1] = np.any(ij[1:] != ij[:-1], axis=1)  # last of each group = max z
    return {(int(a), int(b)): float(c) for (a, b), c in zip(ij[last], z[last])}


def clusters(cells):
    """8-connected components of a set of (ix, iy) cells."""
    todo = set(cells)
    out = []
    while todo:
        seed = todo.pop()
        comp, q = [seed], deque([seed])
        while q:
            cx, cy = q.popleft()
            for dx, dy in NEIGH8:
                n = (cx + dx, cy + dy)
                if n in todo:
                    todo.remove(n)
                    comp.append(n)
                    q.append(n)
        out.append(comp)
    return out


def measure(comp, tops, cell, top_band):
    xy = (np.array(comp, dtype=np.float64) + 0.5) * cell
    h = np.array([tops[c] for c in comp])
    centre = xy.mean(axis=0)
    if len(xy) >= 3:
        cov = np.cov((xy - centre).T)
        _, vecs = np.linalg.eigh(cov)
        axis = vecs[:, -1]
    else:
        axis = np.array([1.0, 0.0])
    perp = np.array([-axis[1], axis[0]])
    d = xy - centre
    along, across = d @ axis, d @ perp
    length = along.max() - along.min() + cell
    width = across.max() - across.min() + cell
    hmax = h.max() + cell / 2.0  # tops are voxel centres
    top_frac = float(np.mean(h >= h.max() - top_band))
    return {
        'x': float(centre[0]), 'y': float(centre[1]),
        'height': float(hmax), 'length': float(length), 'width': float(width),
        'top_fraction': top_frac, 'cells': len(comp),
        'yaw': float(np.arctan2(axis[1], axis[0])),
    }


def _inside(v, rng):
    return rng[0] <= v <= rng[1]


def classify(m, P):
    if m['cells'] < P['min_cells']:
        return None
    if (_inside(m['height'], P['lying_h']) and _inside(m['length'], P['lying_len'])
            and _inside(m['width'], P['lying_wid'])):
        return 'lying'
    if (_inside(m['height'], P['sitting_h']) and _inside(m['length'], P['sitting_len'])
            and _inside(m['width'], P['sitting_wid'])
            and m['top_fraction'] <= P['max_top_fraction']):
        return 'sitting'
    return None


def detect(pts, params=None):
    """pts: Nx3 map-frame voxel centres. Returns (victims, all_candidates); each is a list of dicts."""
    P = dict(DEFAULT_PARAMS)
    if params:
        P.update(params)
    tops = column_tops(pts, P['cell'], P['floor_cut'], P['z_consider_max'])
    structure = {c for c, z in tops.items() if z > P['max_object_height']}
    low = [c for c in tops if c not in structure]
    victims, cands = [], []
    for comp in clusters(low):
        touches = any((cx + dx, cy + dy) in structure for cx, cy in comp for dx, dy in NEIGH8)
        m = measure(comp, tops, P['cell'], P['top_band'])
        m['touches_structure'] = touches
        m['label'] = None if touches else classify(m, P)
        if m['cells'] >= 3:
            cands.append(m)
        if m['label']:
            victims.append(m)
    return victims, cands


class VictimTracker:
    """Confirms a detection after min_hits consecutive updates; confirmed victims are never removed."""

    def __init__(self, match_radius=0.7, min_hits=3):
        self.r = match_radius
        self.min_hits = min_hits
        self.tracks = []      # dicts: x, y, label, hits, confirmed, id
        self.next_id = 1

    def update(self, detections):
        newly = []
        seen = set()
        for d in detections:
            best, bd = None, self.r
            for i, t in enumerate(self.tracks):
                dist = np.hypot(t['x'] - d['x'], t['y'] - d['y'])
                if dist < bd:
                    best, bd = i, dist
            if best is None:
                self.tracks.append({'x': d['x'], 'y': d['y'], 'label': d['label'], 'hits': 1,
                                    'confirmed': False, 'id': None})
                seen.add(len(self.tracks) - 1)
                continue
            t = self.tracks[best]
            seen.add(best)
            a = 0.3  # smooth the position
            t['x'] = (1 - a) * t['x'] + a * d['x']
            t['y'] = (1 - a) * t['y'] + a * d['y']
            t['label'] = d['label']
            t['hits'] += 1
            if not t['confirmed'] and t['hits'] >= self.min_hits:
                t['confirmed'] = True
                t['id'] = self.next_id
                self.next_id += 1
                newly.append(t)
        # unconfirmed tracks that were not seen this round restart
        self.tracks = [t for i, t in enumerate(self.tracks) if t['confirmed'] or i in seen]
        return newly

    def confirmed(self):
        return sorted([t for t in self.tracks if t['confirmed']], key=lambda t: t['id'])
