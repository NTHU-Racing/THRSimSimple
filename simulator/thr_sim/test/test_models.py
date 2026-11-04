import math

import numpy as np

import random

from thr_sim.bicycle_model import BicycleModel, ColoredNoise, PIActuator
from thr_sim.path_utils import make_path, PathProjector


def test_straight_drive():
    # Start already at the commanded speed: the PI loop holds it exactly.
    m = BicycleModel()
    m.reset(v=2.0)
    for _ in range(100):
        m.step(2.0, 0.0, 0.01)
    assert math.isclose(m.state.x, 2.0, abs_tol=1e-6)
    assert math.isclose(m.state.y, 0.0, abs_tol=1e-9)


def test_constant_steer_follows_circle():
    # Turning radius R = L / tan(delta); after driving a full loop we are back.
    L, delta, v = 2.5, 0.3, 3.0
    m = BicycleModel(wheelbase=L)
    m.reset(v=v, steer=delta)
    R = L / math.tan(delta)
    dt = 0.001
    steps = int(round(2 * math.pi * R / v / dt))
    for _ in range(steps):
        m.step(v, delta, dt)
        assert math.isclose(math.hypot(m.state.x, m.state.y - R), R, rel_tol=1e-4)
    assert math.hypot(m.state.x, m.state.y) < 0.05


def test_steer_saturation():
    m = BicycleModel(max_steer=0.5, max_steer_rate=10.0)
    for _ in range(300):
        m.step(1.0, 2.0, 0.01)
        assert m.state.steer <= 0.5
    assert m.state.steer == 0.5


def test_pi_step_response_no_steady_state_error():
    pi = PIActuator(kp=4.8, ki=9.0, max_rate_pos=1e9, max_rate_neg=1e9, max_value=100)
    for _ in range(1000):   # 10 s
        pi.step(5.0, 0.01)
    assert math.isclose(pi.value, 5.0, abs_tol=1e-3)


def test_pi_respects_rate_limits():
    pi = PIActuator(kp=4.8, ki=9.0, max_rate_pos=3.0, max_rate_neg=6.0, max_value=100)
    prev, dt = 0.0, 0.01
    for _ in range(300):
        v = pi.step(5.0, dt)
        assert v - prev <= 3.0 * dt + 1e-12
        prev = v
    for _ in range(300):
        v = pi.step(0.0, dt)
        assert prev - v <= 6.0 * dt + 1e-12
        prev = v


def test_pi_anti_windup_limits_overshoot():
    # Long rate-limited ramp: without anti-windup the integral would build up
    # and cause a large overshoot.
    pi = PIActuator(kp=16.0, ki=100.0, max_rate_pos=0.3, max_rate_neg=0.3, max_value=0.6)
    peak = 0.0
    for _ in range(500):
        peak = max(peak, pi.step(0.5, 0.01))
    assert peak < 0.5 * 1.05
    assert math.isclose(pi.value, 0.5, abs_tol=1e-3)


def test_colored_noise_has_requested_std():
    n = ColoredNoise(std=0.2, tau=0.2, rng=random.Random(1))
    xs = np.array([n.step(0.01) for _ in range(200000)])
    assert abs(xs.mean()) < 0.02
    assert math.isclose(xs.std(), 0.2, rel_tol=0.1)


def test_noise_seed_repeats_after_reset():
    def run(m):
        m.reset()
        for _ in range(500):
            m.step(5.0, 0.1, 0.01)
        return (m.state.x, m.state.y, m.state.yaw)

    m = BicycleModel(speed_noise_std=0.3, steer_noise_std=0.02, seed=7)
    first, second = run(m), run(m)
    assert first == second
    assert first != run(BicycleModel())   # noise actually changes the result


def test_no_creep_when_stopped_with_noise():
    m = BicycleModel(speed_noise_std=0.5, steer_noise_std=0.05, seed=3)
    for _ in range(1000):
        m.step(0.0, 0.0, 0.01)
    assert m.state.x == 0.0 and m.state.y == 0.0


def test_projector_signed_cte_and_progress():
    pts, _ = make_path('straight', length=10.0, ds=0.5, runout=0.0)
    proj = PathProjector(pts)
    s, cte, _ = proj.project(3.2, 1.0)
    assert math.isclose(s, 3.2, abs_tol=1e-9)
    assert math.isclose(cte, 1.0)  # left of path is positive
    _, cte, _ = proj.project(3.4, -0.5)
    assert math.isclose(cte, -0.5)


def test_circle_closed_loop_progress_monotonic():
    pts, _ = make_path('circle', radius=10.0, ds=0.5, runout=0.0)
    proj = PathProjector(pts)
    last = -1.0
    for th in np.linspace(0, 2 * math.pi, 200):
        s, _, _ = proj.project(10 * math.sin(th), 10 * (1 - math.cos(th)))
        assert s >= last - 1e-6
        last = s
    assert last > 0.99 * proj.length


def test_finish_node_splits_path_in_two_segments():
    from thr_sim.path_utils import cumulative_length
    for shape in ('straight', 'wave', 'lane_change'):
        pts, fin = make_path(shape, length=60.0, ds=0.5, runout=20.0)
        assert math.isclose(pts[fin, 0], 60.0)          # finish at x = length
        assert math.isclose(pts[-1, 0], 80.0)           # run-out continues 20 m in x
    pts, fin = make_path('circle', radius=15.0, ds=0.5, runout=20.0)
    s = cumulative_length(pts)
    assert np.hypot(*pts[fin]) < 1e-9                   # one loop: back at the start
    assert math.isclose(s[-1] - s[fin], 20.0, abs_tol=0.5)
