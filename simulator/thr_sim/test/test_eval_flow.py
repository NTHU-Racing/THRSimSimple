"""Evaluation scheduling regressions without starting a ROS graph."""
import queue
import threading
from types import SimpleNamespace

from thr_sim.sim_ctrl_node import SimCtrlNode, IDLE, RUNNING, DONE


class Harness:
    _handle_command = SimCtrlNode._handle_command
    _discard_run = SimCtrlNode._discard_run
    _cancel_evaluation = SimCtrlNode._cancel_evaluation
    _start_session = SimCtrlNode._start_session
    on_tick = SimCtrlNode.on_tick
    request = SimCtrlNode.request

    def __init__(self):
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.eval_shapes = ['straight', 'wave']
        self.eval_runs = self.num_runs = 2
        self.state = IDLE
        self.run_queue = []
        self.run_idx = self.run_total = self.session = 0
        self.started = self.evaluating = self.pending_eval = self.finished = False
        self.auto_start = False
        self.eval_mode = True
        self.eval_phase = 'READY'
        self.next_run_at = None
        self.results = []
        self.report_path = self.report_error = ''
        self.ready = False
        self.no_task = self.saved = self.launches = 0

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)

    def publish_no_task(self):
        self.no_task += 1

    def _save_session_report(self):
        self.saved += 1

    def _ready(self):
        return self.ready

    def _start_next(self):
        self.shape = self.run_queue.pop(0)
        self.run_idx += 1
        self.launches += 1
        self.next_run_at = None
        self.eval_phase = 'RUNNING'
        self.state = RUNNING


def test_click_waits_for_driver_and_locks_in_custom_plan():
    n = Harness()
    n._handle_command('evaluate', {'shapes': ['wave'], 'num_runs': 3})
    n.on_tick()
    assert n.pending_eval and n.eval_phase == 'WAITING'
    assert n.launches == 0 and n.run_total == 3
    n.ready = True
    n.on_tick()
    assert n.shape == 'wave' and n.run_queue == ['wave', 'wave']
    assert n.eval_phase == 'RUNNING' and n.session == 1


def test_empty_plan_does_not_start():
    n = Harness()
    n._handle_command('evaluate', {'shapes': [], 'num_runs': 1})
    assert not n.pending_eval and n.launches == 0


def test_cancel_running_keeps_only_completed_results():
    n = Harness()
    n.evaluating, n.eval_phase, n.state = True, 'RUNNING', RUNNING
    n.results = [{'run': 1}]
    n.run_queue = ['wave']
    n._handle_command('stop', {})
    assert n.results == [{'run': 1}]
    assert n.eval_phase == 'CANCELLED' and n.state == DONE
    assert n.run_queue == [] and n.next_run_at is None
    assert n.no_task == n.saved == 1


def test_cancel_waiting_never_auto_starts_later():
    n = Harness()
    n._handle_command('evaluate', {})
    n._handle_command('stop', {})
    n.ready = True
    n.on_tick()
    assert n.eval_phase == 'CANCELLED' and n.launches == 0


def test_between_tests_waits_for_deadline_and_connection(monkeypatch):
    n = Harness()
    n.evaluating, n.eval_phase, n.state = True, 'BETWEEN', DONE
    n.run_queue = ['wave']
    n.next_run_at = 12.0
    monkeypatch.setattr('thr_sim.sim_ctrl_node.time.monotonic', lambda: 11.0)
    n.on_tick()
    assert n.launches == 0
    monkeypatch.setattr('thr_sim.sim_ctrl_node.time.monotonic', lambda: 12.0)
    n.on_tick()
    assert n.launches == 0  # driver is still unavailable
    n.ready = True
    n.on_tick()
    assert n.launches == 1 and n.shape == 'wave'


def test_rerun_clears_previous_results():
    n = Harness()
    n.evaluating, n.eval_phase, n.state = True, 'CANCELLED', DONE
    n.results = [{'run': 1}]
    n._handle_command('evaluate', {'shapes': ['straight'], 'num_runs': 1})
    assert n.results == [] and n.eval_phase == 'WAITING'
    n.ready = True
    n.on_tick()
    assert n.run_idx == 1 and n.run_total == 1


def test_headless_auto_start_still_runs_without_click():
    n = Harness()
    n.auto_start = True
    n.ready = True
    n.on_tick()  # queue evaluation automatically
    assert n.started
    n.on_tick()  # capture the plan and start
    assert n.launches == 1 and n.eval_phase == 'RUNNING'


def test_previous_pose_cannot_advance_new_path_while_reset_is_pending():
    from thr_sim.sim_ctrl_node import SETTLING
    n = Harness()
    n.state = SETTLING
    n._is_post_reset = lambda msg: False
    n.proj = SimpleNamespace(idx=0)
    previous_pose = SimpleNamespace(x=60.0, y=4.0, yaw=0.0)
    SimCtrlNode.on_state(n, previous_pose)
    assert n.proj.idx == 0 and n.state == SETTLING


def test_run_start_resets_projector_before_first_scored_pose():
    n = Harness()
    resets = []
    n.proj = SimpleNamespace(reset=lambda: resets.append(True))
    n.now_s = lambda: 123.0
    SimCtrlNode._begin_running(n)
    assert resets == [True] and n.t_start == 123.0 and n.state == RUNNING
