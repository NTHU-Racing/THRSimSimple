"""Reference path generation and projection helpers, ROS-independent."""
import math

import numpy as np


def make_path(shape: str, length: float = 60.0, ds: float = 0.5,
              amplitude: float = 3.0, wavelength: float = 30.0,
              radius: float = 15.0, lane_width: float = 4.0,
              runout: float = 20.0):
    """Return (pts, finish_index): an (N, 2) array of path nodes starting at
    (0, 0) heading +x, and the index of the finish (transition) node.

    The path continues `runout` metres past the finish node (after-finish
    segment) so overshoot can be measured and followed. `length` is the x
    extent of the segment before the finish (one full loop for `circle`).
    """
    if shape == 'circle':
        # counter-clockwise loop centred at (0, R) starting at the origin;
        # the finish is back at the origin after one loop
        n_loop = max(8, int(round(2.0 * math.pi * radius / ds)))
        dth = 2.0 * math.pi / n_loop
        n_run = int(round(runout / (radius * dth)))
        th = np.arange(n_loop + n_run + 1) * dth
        pts = np.stack([radius * np.sin(th), radius * (1.0 - np.cos(th))], axis=1)
        return pts, n_loop

    n_fin = int(round(length / ds))
    xs = np.arange(n_fin + int(round(runout / ds)) + 1) * ds
    if shape == 'straight':
        ys = np.zeros_like(xs)
    elif shape == 'wave':
        # y = A/2 * (1 - cos(2*pi*x/lambda)): zero heading at the start
        ys = 0.5 * amplitude * (1.0 - np.cos(2.0 * math.pi * xs / wavelength))
    elif shape == 'lane_change':
        # straight 1/3, smooth lateral shift 1/3, straight 1/3 (+ run-out)
        x0, lc = length / 3.0, length / 3.0
        u = np.clip((xs - x0) / lc, 0.0, 1.0)
        ys = 0.5 * lane_width * (1.0 - np.cos(math.pi * u))
    else:
        raise ValueError(f'unknown path shape: {shape!r}')
    return np.stack([xs, ys], axis=1), n_fin


def cumulative_length(pts: np.ndarray) -> np.ndarray:
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def path_yaws(pts: np.ndarray) -> np.ndarray:
    d = np.diff(pts, axis=0)
    yaw = np.arctan2(d[:, 1], d[:, 0])
    return np.append(yaw, yaw[-1])


class PathProjector:
    """Projects points onto a polyline, tracking a moving index so that
    self-overlapping paths (e.g. a closed loop) are followed in order."""

    def __init__(self, pts: np.ndarray, back_window: int = 5,
                 fwd_window: int = 60):
        self.pts = np.asarray(pts, dtype=float)
        self.s = cumulative_length(self.pts)
        self.length = float(self.s[-1])
        self.back = back_window
        self.fwd = fwd_window
        self.idx = 0

    def reset(self):
        self.idx = 0

    def project(self, x: float, y: float):
        """Return (arc_length, signed_cte, segment_index).
        Positive cte means the point is left of the path."""
        n_seg = len(self.pts) - 1
        lo = max(0, self.idx - self.back)
        hi = min(n_seg, self.idx + self.fwd)
        a = self.pts[lo:hi]
        b = self.pts[lo + 1:hi + 1]
        ab = b - a
        ap = np.array([x, y]) - a
        ab2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12)
        t = np.clip(np.einsum('ij,ij->i', ap, ab) / ab2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d2 = np.sum((proj - np.array([x, y])) ** 2, axis=1)
        k = int(np.argmin(d2))
        i = lo + k
        self.idx = i
        cross = ab[k, 0] * ap[k, 1] - ab[k, 1] * ap[k, 0]
        cte = math.copysign(math.sqrt(d2[k]), cross)
        s = self.s[i] + t[k] * math.sqrt(ab2[k])
        return float(s), float(cte), i

    def point_at(self, s: float):
        """Interpolated (x, y) at arc length s (clamped to the path)."""
        s = min(max(s, 0.0), self.length)
        return (float(np.interp(s, self.s, self.pts[:, 0])),
                float(np.interp(s, self.s, self.pts[:, 1])))
