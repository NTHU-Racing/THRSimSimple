import math

import numpy as np

from thr_driver.path_tracker import PathTracker, wrap_angle


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2)


def test_project_and_point_at_on_straight_path():
    xs = np.arange(0.0, 10.5, 0.5)
    tr = PathTracker(np.stack([xs, np.zeros_like(xs)], axis=1))
    s, cte, _ = tr.project(3.2, 1.0)
    assert math.isclose(s, 3.2, abs_tol=1e-9)
    assert math.isclose(cte, 1.0)
    assert tr.point_at(4.0) == (4.0, 0.0)
    assert tr.point_at(99.0) == (10.0, 0.0)   # clamped to the end


def test_closed_loop_progress_is_monotonic():
    th = np.linspace(0.0, 2 * math.pi, 126)
    tr = PathTracker(np.stack([10 * np.sin(th), 10 * (1 - np.cos(th))], axis=1))
    last = -1.0
    for a in np.linspace(0, 2 * math.pi, 200):
        s, _, _ = tr.project(10 * math.sin(a), 10 * (1 - math.cos(a)))
        assert s >= last - 1e-6
        last = s
    assert last > 0.99 * tr.length
