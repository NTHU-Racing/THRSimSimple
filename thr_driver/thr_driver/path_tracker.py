"""Path tracking helpers for the driver (independent of the simulator package)."""
import math

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PathTracker:
    """Follows progress along a polyline path.

    The nearest segment is searched in a window around the last match, so
    self-overlapping paths (e.g. a closed loop) are followed in order.
    """

    def __init__(self, pts, back_window: int = 5, fwd_window: int = 60):
        self.pts = np.asarray(pts, dtype=float)
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])
        self.back = back_window
        self.fwd = fwd_window
        self.idx = 0

    def reset(self):
        self.idx = 0

    def project(self, x: float, y: float):
        """Return (arc_length, signed_cte, segment_index) of the closest point.
        Positive cte means (x, y) is left of the path."""
        lo = max(0, self.idx - self.back)
        hi = min(len(self.pts) - 1, self.idx + self.fwd)
        a = self.pts[lo:hi]
        ab = self.pts[lo + 1:hi + 1] - a
        ap = np.array([x, y]) - a
        ab2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12)
        t = np.clip(np.einsum('ij,ij->i', ap, ab) / ab2, 0.0, 1.0)
        d2 = np.sum((a + t[:, None] * ab - np.array([x, y])) ** 2, axis=1)
        k = int(np.argmin(d2))
        self.idx = lo + k
        cross = ab[k, 0] * ap[k, 1] - ab[k, 1] * ap[k, 0]
        cte = math.copysign(math.sqrt(d2[k]), cross)
        return float(self.s[self.idx] + t[k] * math.sqrt(ab2[k])), float(cte), self.idx

    def point_at(self, s: float):
        """Interpolated (x, y) at arc length s (clamped to the path)."""
        s = min(max(s, 0.0), self.length)
        return (float(np.interp(s, self.s, self.pts[:, 0])),
                float(np.interp(s, self.s, self.pts[:, 1])))
