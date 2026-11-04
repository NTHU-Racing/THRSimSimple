import math

from demo_driver.driver_node import control_command


def test_no_task_or_feedback_means_stop():
    assert control_command([], 0, (0, 0, 0), 0)[:2] == (0, 0)
    assert control_command([(0, 0), (1, 0)], 1, None, 0)[:2] == (0, 0)


def test_straight_path_constant_speed():
    v, steer, _ = control_command([(i, 0) for i in range(20)], 15, (0, 0, 0), 0)
    assert v == 3 and steer == 0


def test_positive_error_turns_left_and_is_limited():
    _, steer, _ = control_command([(0, i) for i in range(20)], 15, (0, 0, 0), 0)
    assert steer == 0.6


def test_circle_does_not_finish_at_start():
    points = [(10 * math.sin(i * math.pi / 50), 10 * (1 - math.cos(i * math.pi / 50)))
              for i in range(120)]
    v, _, index = control_command(points, 100, (0, 0, 0), 0)
    assert v == 3 and index < 100


def test_finish_stops_and_progress_never_goes_backwards():
    points = [(i, 0) for i in range(20)]
    v, steer, index = control_command(points, 10, (10, 0, 0), 8)
    assert v == steer == 0 and index >= 10
    _, _, new_index = control_command(points, 10, (0, 0, 0), index)
    assert new_index >= index
