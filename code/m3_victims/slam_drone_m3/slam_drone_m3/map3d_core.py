"""Pure-numpy helpers for the 3D height-coloured map (no ROS imports, so they can be unit tested)."""
import numpy as np

# Voxel keys are packed into one int64: 21 bits per axis, offset so negatives work.
_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1


def quat_to_rot(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def filter_sensor_points(pts, min_range, max_range):
    """Drop NaN/inf returns, points on the drone itself (< min_range) and far noisy points."""
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    r = np.linalg.norm(pts, axis=1)
    return pts[(r >= min_range) & (r <= max_range)]


def transform_points(pts, rot, trans):
    return pts @ rot.T + np.asarray(trans)


def voxel_keys(pts, voxel):
    idx = np.floor(pts / voxel).astype(np.int64) + _OFF
    idx = np.clip(idx, 0, _MASK)
    return (idx[:, 0] << (2 * _BITS)) | (idx[:, 1] << _BITS) | idx[:, 2]


def keys_to_centers(keys, voxel):
    ix = (keys >> (2 * _BITS)) & _MASK
    iy = (keys >> _BITS) & _MASK
    iz = keys & _MASK
    idx = np.stack([ix, iy, iz], axis=1) - _OFF
    return (idx.astype(np.float64) + 0.5) * voxel


class VoxelMap:
    """Accumulates points into a sparse voxel set with a hit count per voxel."""

    def __init__(self, voxel=0.1, z_min=-0.2, z_max=3.2):
        self.voxel = voxel
        self.z_min = z_min
        self.z_max = z_max
        self.keys = np.zeros(0, dtype=np.int64)
        self.hits = np.zeros(0, dtype=np.int64)

    def clear(self):
        self.keys = np.zeros(0, dtype=np.int64)
        self.hits = np.zeros(0, dtype=np.int64)

    def insert(self, pts_map):
        pts = pts_map[(pts_map[:, 2] >= self.z_min) & (pts_map[:, 2] <= self.z_max)]
        if len(pts) == 0:
            return
        # one hit per voxel per scan, so a single scan can't make noise look solid
        new = np.unique(voxel_keys(pts, self.voxel))
        allk = np.concatenate([self.keys, new])
        allh = np.concatenate([self.hits, np.ones(len(new), dtype=np.int64)])
        self.keys, inv = np.unique(allk, return_inverse=True)
        self.hits = np.bincount(inv, weights=allh).astype(np.int64)

    def points(self, min_hits=2):
        return keys_to_centers(self.keys[self.hits >= min_hits], self.voxel)

    def __len__(self):
        return len(self.keys)


def height_colors(z, z_lo, z_hi):
    """Rainbow by height: blue (low) -> cyan -> green -> yellow -> red (high). Returns uint8 Nx3."""
    t = np.clip((z - z_lo) / max(z_hi - z_lo, 1e-6), 0.0, 1.0)
    h = (1.0 - t) * 4.0  # 4 = blue (floor) ... 0 = red (top of walls)
    # piecewise linear: 0 red(1,0,0) 1 yellow(1,1,0) 2 green(0,1,0) 3 cyan(0,1,1) 4 blue(0,0,1)
    r = np.clip(2.0 - h, 0.0, 1.0)
    g = np.where(h < 1.0, h, np.where(h < 3.0, 1.0, 4.0 - h))
    g = np.clip(g, 0.0, 1.0)
    b = np.clip(h - 2.0, 0.0, 1.0)
    return (np.stack([r, g, b], axis=1) * 255.0).astype(np.uint8)


def pack_xyzrgb(pts, rgb):
    """Build PointCloud2 data bytes for fields x, y, z, rgb (all float32, point_step 16)."""
    n = len(pts)
    arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    arr['x'], arr['y'], arr['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    rgb = rgb.astype(np.uint32)
    arr['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    return arr.tobytes()


def write_ply(path, pts, rgb):
    """ASCII PLY with colours, opens in CloudCompare / MeshLab on Windows for report figures."""
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(pts)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p, c in zip(pts, rgb):
            f.write(f'{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} {c[0]} {c[1]} {c[2]}\n')
