"""Simulation control node: publishes the reference path, resets the
simulation and scores how well the vehicle follows the path and stops at the
finish node.

Publishes:
    /path       (thr_sim_msgs/RefPath, transient-local) path nodes + finish index;
                an empty path means "no task" (between runs / after Stop)
    /reset      (std_msgs/Empty)                 start of each run
    /score      (std_msgs/Float64)               final score of each run
    /sim_status (std_msgs/String)                JSON run result
Subscribes:
    /veh_state  (thr_sim_msgs/VehState)
    /veh_ctrl   (thr_sim_msgs/VehCtrl)           only for display (commanded)
    /veh_actual (thr_sim_msgs/VehCtrl)           only for display (actual)

The path has two segments: drive to the finish (transition) node, stop there;
the after-finish run-out is only there to absorb overshoot. A run ends when
    stopped    the vehicle has been at rest for `stop_hold` s after setting off
    off_track  |cross-track error| > max_cte
    overrun    the vehicle reached the end of the run-out
    timeout    time_limit exceeded
    aborted    the user pressed Stop / Reset / changed the path
Score (max 100) = tracking points + time points + stop points, with maxima
100 * tracking_weight, 100 * time_weight and the rest (default 40 / 40 / 20):
    tracking = completion * max(0, 1 - rms_cte / max_cte)
        completion = progress up to the finish node / distance to the finish node
    time = min(v_eq, time_v_max) / time_v_max, linear in the
        equivalent speed v_eq = distance to the finish node / time to finish
        (0 unless the run ended `stopped` after reaching the finish)
    stop points = full within stop_free_zone of the finish node, minus
        stop_penalty points per metre beyond it, >= 0
        (distance measured along the path; 0 unless the run ended `stopped`)

Evaluate mode runs every shape in `eval_shapes` `num_runs` times, prints a
summary and optionally writes it to `results_file` (JSON).

With `gui:=true` a Tk window shows the map and lets you start, evaluate, stop
and reset runs; otherwise runs start automatically.
"""
import json
import math
import os
import queue
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Empty, Float64, String

from thr_sim_msgs.msg import RefPath, VehCtrl, VehState
from thr_sim.path_utils import make_path, PathProjector

IDLE, SETTLING, RUNNING, DONE = 'IDLE', 'SETTLING', 'RUNNING', 'DONE'
PATH_SHAPES = ('straight', 'wave', 'lane_change', 'circle')
MOVED_DIST = 1.0   # stop detection starts once the vehicle got this far [m]


class SimCtrlNode(Node):
    def __init__(self):
        super().__init__('sim_ctrl')
        p = self.declare_parameter
        self.shape = p('path_shape', 'wave').value
        self.max_cte = p('max_cte', 2.0).value          # off-track limit [m]
        self.time_limit = p('time_limit', 60.0).value   # per run [s]
        self.num_runs = p('num_runs', 1).value
        # auto runs / evaluation wait for the background driver (a /path subscriber)
        self.wait_driver = p('wait_for_driver', True).value
        self.settle_time = p('settle_time', 0.5).value
        self.shutdown_on_finish = p('shutdown_on_finish', False).value
        self.auto_start = p('auto_start', True).value
        self.stop_hold = p('stop_hold', 1.0).value      # at rest this long = stopped [s]
        self.stop_eps = p('stop_eps', 0.05).value       # "at rest": moved less than [m]
        self.stop_free = p('stop_free_zone', 5.0).value     # full stop points within [m]
        self.stop_penalty = p('stop_penalty', 5.0).value    # points lost per m beyond it
        self.w_track = p('tracking_weight', 0.4).value  # share of the 100 points
        self.w_time = p('time_weight', 0.4).value       # stop gets 1 - both
        self.w_stop = max(0.0, 1.0 - self.w_track - self.w_time)
        self.time_v_max = p('time_v_max', 20.0).value   # full time points at this v_eq [m/s]
        self.eval_mode = p('eval_mode', False).value
        self.inter_run_delay = max(0.0, p('inter_run_delay', 2.0).value)
        self.eval_shapes = [s.strip() for s in p('eval_shapes', ','.join(PATH_SHAPES)).value
                            .split(',') if s.strip()]
        self.results_file = p('results_file', '').value
        self.path_kwargs = dict(
            length=p('path_length', 60.0).value,
            ds=p('path_ds', 0.5).value,
            amplitude=p('wave_amplitude', 3.0).value,
            wavelength=p('wave_length', 30.0).value,
            radius=p('circle_radius', 15.0).value,
            lane_width=p('lane_width', 4.0).value,
            runout=p('runout_length', 20.0).value,
        )
        for s in [self.shape] + self.eval_shapes:
            if s not in PATH_SHAPES:
                raise ValueError(f'unknown path shape {s!r}, use one of {PATH_SHAPES}')

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(RefPath, '/path', latched)
        self.reset_pub = self.create_publisher(Empty, '/reset', 10)
        self.score_pub = self.create_publisher(Float64, '/score', 10)
        self.status_pub = self.create_publisher(String, '/sim_status', 10)
        self.create_subscription(VehState, '/veh_state', self.on_state, 10)
        self.create_subscription(VehCtrl, '/veh_ctrl', self.on_ctrl, 10)
        self.create_subscription(VehCtrl, '/veh_actual', self.on_actual, 10)

        # Shared with the GUI thread; guarded by self.lock.
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.state = IDLE
        self.run_queue = []        # shapes still to run in this session
        self.run_total = 0         # runs in this session (for "run i / n")
        self.run_idx = 0
        self.evaluating = False
        self.eval_runs = self.num_runs   # runs per path of the (last) evaluation
        self.pending_eval = False        # evaluation requested, waiting for the driver
        self.session = 0                 # increments with every Start / evaluation
        self.eval_phase = 'READY'        # READY / WAITING / RUNNING / BETWEEN / COMPLETE / CANCELLED
        self.next_run_at = None          # monotonic deadline for the next evaluation run
        self.report_path = ''
        self.report_error = ''
        self.started = False
        self.results = []
        self.veh = None            # (x, y, yaw)
        self.ctrl = None           # commanded (velocity, steer)
        self.act = None            # actual (velocity, steer)
        # Recorded all the time (not only during runs) for the UI; times are
        # plot_time(). The plots show the last 10 s, so ~40 s of history is kept.
        self.trail = deque(maxlen=20000)      # (x, y), cleared at run start / reset
        self.cte_hist = deque(maxlen=2000)    # (t, cte)
        self.ctrl_hist = deque(maxlen=2000)   # (t, velocity, steer), commanded
        self.act_hist = deque(maxlen=2000)    # (t, velocity, steer), actual
        self.cur = {}              # live stats of the current run
        self.finished = False
        self._reset_run_stats()
        self.set_path(self.shape)
        self.publish_no_task()     # nothing to drive until a run starts

        self.create_timer(0.05, self.on_tick)
        if self.eval_mode and self.auto_start:
            hint = (f'Evaluating {self.eval_shapes} x {self.num_runs} run(s) once '
                    'vehicle_sim and a driver are up.')
        elif self.eval_mode:
            hint = 'Press "Start evaluation" in the UI.'
        elif self.auto_start:
            hint = 'Runs start once vehicle_sim and a driver are up.'
        else:
            hint = 'Press Start in the UI to run.'
        self.get_logger().info(
            f"Path '{self.shape}', {self.s_finish:.1f} m to the finish. {hint}")

    # ------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_path(self, shape):
        pts, fin = make_path(shape, **self.path_kwargs)
        with self.lock:
            self.shape = shape
            self.proj = PathProjector(pts)
            self.path_pts = pts
            self.finish_idx = fin
            self.s_finish = float(self.proj.s[fin])
        # Published in start_run() so the driver gets nothing before a run.
        self.path_msg = RefPath()
        self.path_msg.header.frame_id = 'map'
        self.path_msg.nodes = [Point(x=float(x), y=float(y)) for x, y in pts]
        self.path_msg.finish_index = int(fin)

    def publish_no_task(self):
        msg = RefPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)

    @staticmethod
    def plot_time():
        return time.monotonic()

    def stop_fraction(self, stop_err):
        """Share (0..1) of the stop points for stopping stop_err m from the finish:
        full within stop_free_zone, then stop_penalty points lost per metre."""
        max_pts = 100.0 * self.w_stop
        if max_pts <= 0.0:
            return 0.0
        pts = max_pts - self.stop_penalty * max(0.0, abs(stop_err) - self.stop_free)
        return max(0.0, pts) / max_pts

    def time_fraction(self, v_eq):
        """Share (0..1) of the time points: linear in v_eq up to time_v_max."""
        return min(max(v_eq, 0.0), self.time_v_max) / self.time_v_max

    def stop_zero_dist(self):
        """Distance from the finish at which the stop points reach 0 [m]."""
        max_pts = 100.0 * self.w_stop
        return self.stop_free + (max_pts / self.stop_penalty if self.stop_penalty > 0 else 0.0)

    def elapsed(self):
        if self.state != RUNNING:
            return self.cur.get('time', 0.0)
        return self.now_s() - self.t_start

    def _reset_run_stats(self):
        self.progress = 0.0
        self.progress_time = 0.0
        self.finish_time = None
        self.sq_cte_sum = 0.0
        self.max_abs_cte = 0.0
        self.n_samples = 0
        self.rest_win = deque()    # (t, x, y) over the last stop_hold seconds
        self.cur = {'progress': 0.0, 'cte': 0.0, 'rms_cte': 0.0, 'time': 0.0,
                    'to_finish': getattr(self, 's_finish', 0.0)}

    def connections(self):
        """Counts used by the UI to show which nodes are up."""
        return {
            'reset_subs': self.reset_pub.get_subscription_count(),
            'state_pubs': self.count_publishers('/veh_state'),
            'ctrl_pubs': self.count_publishers('/veh_ctrl'),
            'path_subs': self.path_pub.get_subscription_count(),
        }

    # ------------------------------------------- commands (from GUI thread)
    def request(self, cmd, **kwargs):
        self.commands.put((cmd, kwargs))

    def _handle_command(self, cmd, kw):
        if cmd == 'start':      # num_runs runs on one shape
            self.pending_eval = False
            shape = kw.get('shape') or self.shape
            self._start_session([shape] * int(kw.get('num_runs', self.num_runs)), False)
        elif cmd == 'evaluate':  # num_runs runs on every eval shape
            shapes = list(kw.get('shapes', self.eval_shapes))
            runs = int(kw.get('num_runs', self.num_runs))
            if not shapes or any(shape not in PATH_SHAPES for shape in shapes) or runs < 1:
                self.get_logger().warn('Select at least one valid path and at least one repeat.')
                return
            self._discard_run()
            self.publish_no_task()
            with self.lock:
                self.eval_shapes = list(dict.fromkeys(shapes))
                self.eval_runs = runs
                self.evaluating = True
                self.results = []
                self.run_idx = 0
                self.run_total = len(self.eval_shapes) * runs
                self.run_queue = []
                self.state = IDLE
                self.eval_phase = 'WAITING'
                self.report_path = self.report_error = ''
            self.pending_eval = True   # user requested; wait for the background driver
        elif cmd == 'stop':
            if self.evaluating and self.eval_phase in ('WAITING', 'RUNNING', 'BETWEEN'):
                self._cancel_evaluation()
            else:
                self.pending_eval = False
                self._abort()
                self.publish_no_task()
        elif cmd == 'reset':
            self._reset_sim()
        elif cmd == 'shape':
            # Switching path stops any run and puts the car back at the start.
            self._reset_sim()
            self.set_path(kw['shape'])
            self.get_logger().info(f"Path switched to '{kw['shape']}'. Press Start to run.")

    def _discard_run(self):
        """Drop the current session without scoring the run in progress
        (used by a fresh Start / Run again)."""
        self.run_queue = []
        self.next_run_at = None
        if self.state in (SETTLING, RUNNING):
            with self.lock:
                self.state = IDLE
            self.get_logger().info(f'Run {self.run_idx} discarded without scoring.')

    def _cancel_evaluation(self):
        # Only completed tests count. Cancelling never adds an 'aborted' score.
        self.pending_eval = False
        self._discard_run()
        with self.lock:
            self.state = DONE
            self.eval_phase = 'CANCELLED'
        self.publish_no_task()
        self._save_session_report()
        self.get_logger().info('Evaluation cancelled; completed results retained.')

    def _save_session_report(self):
        if not self.results:
            return
        try:
            self._report()
        except Exception as exc:
            self.report_error = str(exc)
            self.get_logger().error(f'Could not save evaluation results: {exc}')

    def _abort(self):
        self.run_queue = []
        if self.state in (SETTLING, RUNNING):
            self.finish_run('aborted')

    def _reset_sim(self):
        if self.evaluating and self.eval_phase in ('WAITING', 'RUNNING', 'BETWEEN'):
            self._cancel_evaluation()
        else:
            self._abort()
        self.pending_eval = False
        self.next_run_at = None
        self.publish_no_task()
        self.reset_pub.publish(Empty())
        with self.lock:
            self.state = IDLE
            self.proj.reset()
            self.trail.clear()
            self._reset_run_stats()

    # ------------------------------------------------------------ run cycle
    def _start_session(self, shapes, evaluating):
        self._discard_run()
        with self.lock:
            self.results = []
            self.run_queue = list(shapes)
            self.run_total = len(shapes)
            self.run_idx = 0
            self.evaluating = evaluating
        self.started = True
        self.session += 1
        self.next_run_at = None
        self.report_path = self.report_error = ''
        self.eval_phase = 'RUNNING' if evaluating else 'READY'
        if evaluating:
            self.get_logger().info(
                f'=== Evaluation: {len(shapes)} runs over {sorted(set(shapes), key=shapes.index)} ===')
        self._start_next()

    def _start_next(self):
        self.next_run_at = None
        if self.evaluating:
            self.eval_phase = 'RUNNING'
        shape = self.run_queue.pop(0)
        if shape != self.shape:
            self.set_path(shape)
        self.run_idx += 1
        self.t_reset = self.now_s()   # before /reset: later states are post-reset
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        self.reset_pub.publish(Empty())
        with self.lock:
            self.proj.reset()
            self.trail.clear()
            self._reset_run_stats()
            self.state = SETTLING
        self.get_logger().info(
            f"=== Run {self.run_idx}/{self.run_total} on '{self.shape}': reset sent ===")

    def on_tick(self):
        while not self.commands.empty():
            self._handle_command(*self.commands.get())

        if self.pending_eval:
            if self._ready():
                self.pending_eval = False
                self._start_session(
                    [s for s in self.eval_shapes for _ in range(self.eval_runs)], True)
        elif self.evaluating and self.eval_phase == 'BETWEEN':
            if time.monotonic() >= self.next_run_at and self._ready():
                self._start_next()
        elif self.state == IDLE and not self.started and self.auto_start:
            # headless: start by itself (with the UI the user clicks Start / Start evaluation)
            if self._ready():
                self.request('evaluate' if self.eval_mode else 'start')
                self.started = True
        elif self.state == SETTLING:
            # Normally on_state starts the run within a few ms; this is a fallback.
            if self.now_s() - self.t_reset >= self.settle_time:
                with self.lock:
                    self._begin_running()
        elif self.state == RUNNING:
            self.cur['time'] = self.elapsed()
            if self.cur['time'] > self.time_limit:
                self.finish_run('timeout')

    def _ready(self):
        """Check vehicle feedback/reset and driver path/control endpoints.
        Logs missing connections every 5 s; wait_for_driver can bypass the driver check."""
        counts = self.connections()
        sim_up = counts['reset_subs'] >= 1 and counts['state_pubs'] >= 1
        driver_up = not self.wait_driver or (counts['path_subs'] >= 1 and counts['ctrl_pubs'] >= 1)
        if sim_up and driver_up:
            return True
        now = self.now_s()
        if now - getattr(self, '_last_wait_log', 0.0) >= 5.0:
            self._last_wait_log = now
            missing = [n for n, up in (('vehicle_sim', sim_up),
                                       ('a driver subscribed to /path and publishing /veh_ctrl', driver_up)) if not up]
            self.get_logger().info(f"Waiting for {' and '.join(missing)} "
                                   f"(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')})...")
        return False

    def on_ctrl(self, msg: VehCtrl):
        with self.lock:
            self.ctrl = (msg.velocity, msg.steer)
            self.ctrl_hist.append((self.plot_time(), msg.velocity, msg.steer))

    def on_actual(self, msg: VehCtrl):
        with self.lock:
            self.act = (msg.velocity, msg.steer)
            self.act_hist.append((self.plot_time(), msg.velocity, msg.steer))

    def _begin_running(self):
        """Call with self.lock held. Start projection from the reset position."""
        self.proj.reset()
        self.t_start = self.now_s()
        self.state = RUNNING

    def _is_post_reset(self, msg: VehState):
        """True for a state sent after the last /reset (vehicle back at the start)."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x0, y0 = self.path_pts[0]
        return stamp >= self.t_reset and math.hypot(msg.x - x0, msg.y - y0) < 0.5

    def _at_rest(self, t, x, y):
        """Update the rest window; True once the vehicle has not moved more than
        stop_eps for stop_hold seconds."""
        w = self.rest_win
        w.append((t, x, y))
        while len(w) > 1 and t - w[1][0] >= self.stop_hold:
            w.popleft()
        if t - w[0][0] < self.stop_hold:
            return False
        return all(math.hypot(x - px, y - py) < self.stop_eps for _, px, py in w)

    def rest_since(self):
        """Run time at which the vehicle came to rest (start of the rest window)."""
        return self.rest_win[0][0] if self.rest_win else self.elapsed()

    def on_state(self, msg: VehState):
        with self.lock:
            self.veh = (msg.x, msg.y, msg.yaw)
            if self.state == SETTLING:
                # Feedback from the previous path can arrive while /reset is
                # in flight. Do not let it advance the new path's projector.
                if not self._is_post_reset(msg):
                    return
                self._begin_running()   # start the clock as soon as the reset took effect
            # display data, also between runs (the car is always driven)
            if not self.trail or math.hypot(msg.x - self.trail[-1][0],
                                            msg.y - self.trail[-1][1]) >= 0.02:
                self.trail.append((msg.x, msg.y))
            s, cte, _ = self.proj.project(msg.x, msg.y)
            self.cte_hist.append((self.plot_time(), cte))
            self.cur['cte'] = cte
            self.cur['to_finish'] = self.s_finish - s
            if self.state != RUNNING:
                return
            # scoring, only during a run
            previous_progress = self.progress
            previous_progress_time = self.progress_time
            self.progress = max(self.progress, s)
            self.sq_cte_sum += cte * cte
            self.max_abs_cte = max(self.max_abs_cte, abs(cte))
            self.n_samples += 1
            t = self.elapsed()
            if self.finish_time is None and self.progress >= self.s_finish:
                if previous_progress < self.s_finish and self.progress > previous_progress:
                    fraction = (self.s_finish - previous_progress) / (
                        self.progress - previous_progress)
                    self.finish_time = previous_progress_time + fraction * (
                        t - previous_progress_time)
                else:
                    self.finish_time = t
            if self.progress > previous_progress:
                self.progress_time = t
            self.cur.update(progress=min(1.0, self.progress / self.s_finish),
                            rms_cte=math.sqrt(self.sq_cte_sum / self.n_samples), time=t)
            stopped = self._at_rest(t, msg.x, msg.y) and self.progress >= MOVED_DIST
            overrun = s >= self.proj.length - 0.5
        if abs(cte) > self.max_cte:
            self.finish_run('off_track')
        elif overrun:
            self.finish_run('overrun', s_stop=s)
        elif stopped:
            self.finish_run('stopped', s_stop=s, t_end=self.rest_since())

    def finish_run(self, reason, s_stop=None, t_end=None):
        elapsed = self.elapsed() if t_end is None else t_end
        rms = math.sqrt(self.sq_cte_sum / self.n_samples) if self.n_samples else 0.0
        completion = min(1.0, self.progress / self.s_finish)
        tracking = completion * max(0.0, 1.0 - rms / self.max_cte)
        stop_err = None if s_stop is None else s_stop - self.s_finish
        stop = timing = v_eq = 0.0
        if reason == 'stopped':
            if self.progress >= self.s_finish:
                stop = self.stop_fraction(stop_err)
            if self.finish_time is not None:
                v_eq = self.s_finish / self.finish_time if self.finish_time > 0 else 0.0
                timing = self.time_fraction(v_eq)

        score = 100.0 * (self.w_track * tracking + self.w_time * timing + self.w_stop * stop)
        result = {
            'run': self.run_idx, 'shape': self.shape, 'result': reason,
            'score': round(score, 2), 'tracking': round(100 * tracking, 2),
            'time_score': round(100 * timing, 2), 'v_eq': round(v_eq, 3),
            'stop': round(100 * stop, 2),
            'stop_err': None if stop_err is None else round(stop_err, 3),
            'completion': round(completion, 4), 'rms_cte': round(rms, 4),
            'max_cte': round(self.max_abs_cte, 4), 'time': round(elapsed, 2),
        }
        with self.lock:
            self.results.append(result)
            self.cur.update(progress=completion, rms_cte=rms, time=elapsed)
            self.state = DONE
        self.score_pub.publish(Float64(data=score))
        self.status_pub.publish(String(data=json.dumps(result)))
        err = '–' if stop_err is None else f'{stop_err:+.2f} m'
        self.get_logger().info(
            f'Run {self.run_idx} finished: {reason} | score {score:.1f}/100 | '
            f'tracking {100 * self.w_track * tracking:.1f}/{100 * self.w_track:.0f} | '
            f'time {100 * self.w_time * timing:.1f}/{100 * self.w_time:.0f} '
            f'(v_eq {v_eq:.2f} m/s) | stop {100 * self.w_stop * stop:.1f}/{100 * self.w_stop:.0f} '
            f'(err {err}) | completion {completion * 100:.1f}% | RMS cte {rms:.3f} m | '
            f'time {elapsed:.2f} s')

        if self.run_queue:
            if self.evaluating:
                # Keep the result visible briefly; physics continues normally.
                self.publish_no_task()
                self.eval_phase = 'BETWEEN'
                self.next_run_at = time.monotonic() + self.inter_run_delay
            else:
                self._start_next()
            return
        if self.evaluating:
            self.eval_phase = 'COMPLETE'
            self.publish_no_task()
            self.get_logger().info('Evaluation finished: no task published, driver should stop.')
        if reason != 'aborted' or len(self.results) > 1:
            self._save_session_report()
        if self.shutdown_on_finish and reason != 'aborted':
            self.finished = True

    @staticmethod
    def summarize(results):
        """Per-path averages (in first-run order) of a list of run results."""
        summary = {}
        for sh in dict.fromkeys(r['shape'] for r in results):
            rs = [r for r in results if r['shape'] == sh]
            errs = [abs(r['stop_err']) for r in rs if r['result'] == 'stopped']
            summary[sh] = {
                'runs': len(rs),
                'score': sum(r['score'] for r in rs) / len(rs),
                'tracking': sum(r['tracking'] for r in rs) / len(rs),
                'time_score': sum(r['time_score'] for r in rs) / len(rs),
                'v_eq': sum(r['v_eq'] for r in rs) / len(rs),
                'stop': sum(r['stop'] for r in rs) / len(rs),
                'mean_abs_stop_err': sum(errs) / len(errs) if errs else None,
                'stopped': len(errs),
                'results': [r['result'] for r in rs],
            }
        return summary

    def _report(self):
        res = self.results
        avg = sum(r['score'] for r in res) / len(res)
        if not self.evaluating:
            self.get_logger().info(f'=== {len(res)} run(s) done. Average score: {avg:.2f} ===')
            return
        summary = self.summarize(res)
        wt, wv, ws = 100 * self.w_track, 100 * self.w_time, 100 * self.w_stop
        lines = ['', f"{'path':<12}{'runs':>5}{'score/100':>10}{f'track/{wt:.0f}':>10}"
                 f"{f'time/{wv:.0f}':>9}{f'stop/{ws:.0f}':>9}{'v_eq':>7}{'|stop err|':>11}  results"]
        for sh, row in summary.items():
            err = '–' if row['mean_abs_stop_err'] is None else f"{row['mean_abs_stop_err']:.2f} m"
            lines.append(f"{sh:<12}{row['runs']:>5}{row['score']:>10.1f}"
                         f"{self.w_track * row['tracking']:>10.1f}{self.w_time * row['time_score']:>9.1f}"
                         f"{self.w_stop * row['stop']:>9.1f}{row['v_eq']:>7.2f}{err:>11}"
                         f"  {', '.join(row['results'])}")
        lines.append(f"{'OVERALL':<12}{len(res):>5}{avg:>10.1f}")
        self.get_logger().info('=== Evaluation results ===' + '\n'.join(lines))
        if self.results_file:
            path = os.path.expanduser(self.results_file)
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            with open(path, 'w') as f:
                json.dump({'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                           'status': self.eval_phase.lower(),
                           'planned_runs': self.run_total, 'completed_runs': len(res),
                           'plan': {'shapes': self.eval_shapes, 'repeats': self.eval_runs},
                           'overall_score': round(avg, 2), 'per_path': summary,
                           'runs': res}, f, indent=2)
            self.report_path = os.path.abspath(path)
            png = os.path.splitext(path)[0] + '.png'
            try:
                from thr_sim.eval_report import save_report_png
                save_report_png(png, res, summary, (self.w_track, self.w_time, self.w_stop),
                                self.stop_free, self.stop_zero_dist())
                self.get_logger().info(f'Results written to {path} and {png}')
            except Exception as e:   # the JSON is the essential part
                self.get_logger().warn(f'Results written to {path}; report PNG failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimCtrlNode()
    use_gui = node.declare_parameter('gui', False).value
    if use_gui and not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        node.get_logger().warn('No display found, running without the UI')
        use_gui = False

    spin_thread = None
    try:
        if use_gui:
            from thr_sim.sim_ctrl_gui import SimCtrlGui

            def spin():
                try:
                    while rclpy.ok() and not node.finished:
                        rclpy.spin_once(node, timeout_sec=0.05)
                except ExternalShutdownException:
                    pass
                except Exception:
                    # Ctrl-C can invalidate the context mid-wait (RCLError);
                    # only report errors that happen while still running.
                    if rclpy.ok() and not node.finished:
                        raise

            spin_thread = threading.Thread(target=spin, daemon=True)
            spin_thread.start()
            SimCtrlGui(node).run()   # blocks until the window is closed
        else:
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finished = True
        # The spin thread must leave spin_once before the node is destroyed,
        # otherwise rclpy aborts ("terminate called without an active exception").
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
