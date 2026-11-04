# THRSimSimple

A minimal ROS 2 (Humble) vehicle simulator in Python using a kinematic bicycle
model. A driver follows a two-segment reference path and has to stop at the
finish node; the simulator scores path tracking and stopping accuracy.

## Quick start

The scripts start only the simulator. The driver runs in the background as
its own node (same `ROS_DOMAIN_ID`) and just reacts to the simulator's topics:

```bash
# terminal 1: your driver, or the demo driver
source install/setup.bash && ros2 launch thr_driver driver.launch.py

# terminal 2: the simulator
./run_sim.sh            # UI: press Start or Evaluate
./evaluate.sh           # evaluation UI: press Start evaluation; every path x 3 runs
```

Both scripts build the workspace first (`--no-build` skips it). Auto runs and
the evaluation wait until a driver subscribes to `/path`, logging every 5 s
while they wait; the driver can keep running across many simulator sessions.

| Script | Options |
|---|---|
| `run_sim.sh` | `--headless`, `--no-build`, plus launch args such as `path_shape:=circle num_runs:=3` |
| `evaluate.sh` | `-n RUNS` (default 3), `-s SHAPES` (default all), `-o FILE` (default `eval_results/eval_<date>_<time>.json`), `--headless` (no UI, exits when done), `--no-build` |

`evaluate.sh` opens the evaluation UI; nothing runs until you press
**▶ Start evaluation** (if the driver is not up yet it waits for it). When all
runs are done the task is taken away (empty `/path`) so the driver stops the
car, and the summary is written to `eval_results/eval_<date>_<time>.json`
(plus a chart `.png` of the same results). The window stays
open; close it to exit. `--headless` starts by itself and exits when done.

## Packages

| Package | Contents | Depends on |
|---|---|---|
| `simulator/thr_sim_msgs` | `VehCtrl` (velocity, steer), `VehState` (x, y, yaw), `RefPath` (nodes, finish index) | – |
| `simulator/thr_sim` | simulator: `vehicle_sim`, `sim_ctrl` (+ UI), `sim.launch.py`, `sim.yaml` | `thr_sim_msgs` |
| `thr_driver` | demo driver: `driver` node, `driver.launch.py`, `demo.launch.py` | `thr_sim_msgs` (+ `thr_sim` only for `demo.launch.py`) |

The simulator and the driver only talk through ROS topics; neither imports
the other's code.

## Simulator I/O (ROS 2 topics)


The simulator consists of `vehicle_sim` (vehicle physics) and `sim_ctrl`
(paths, resets, evaluation and UI). The external driver runs separately on the
same `ROS_DOMAIN_ID`. Directions below are **relative to the simulator**:
input means the driver/user publishes it; output means the simulator publishes it.

### Topic reference

| Direction | Topic | ROS message type | Producer → consumer | When / rate |
|---|---|---|---|---|
| Input | `/veh_ctrl` | `thr_sim_msgs/msg/VehCtrl` | Driver → `vehicle_sim` (also displayed by `sim_ctrl`) | Publish continuously; demo uses 20 Hz |
| Input / output | `/reset` | `std_msgs/msg/Empty` | `sim_ctrl` or user → `vehicle_sim` and driver | Once at each run start or UI reset; users may also publish it |
| Output | `/veh_state` | `thr_sim_msgs/msg/VehState` | `vehicle_sim` → driver and `sim_ctrl` | 50 Hz by default, plus immediately after reset |
| Output | `/veh_actual` | `thr_sim_msgs/msg/VehCtrl` | `vehicle_sim` → UI; optionally driver | Same rate as `/veh_state` |
| Output | `/path` | `thr_sim_msgs/msg/RefPath` | `sim_ctrl` → driver | At run start or when a task is cleared; latest value retained |
| Output | `/score` | `std_msgs/msg/Float64` | `sim_ctrl` → result consumers | Once per finished run; `data` is the total score, 0–100 |
| Output | `/sim_status` | `std_msgs/msg/String` | `sim_ctrl` → result consumers | Once per finished run; `data` is a JSON result, **not** a continuous status feed |

All topics use **reliable** delivery. `/path` uses **transient-local durability**
and keep-last depth 1; use the same QoS in the driver to receive the current task
when connecting late. Other topics use volatile durability and depth 10.

### Message fields, units and coordinates

`/veh_ctrl` — desired speed and front-wheel angle:

```text
std_msgs/Header header
float64 velocity     # m/s; positive forward, negative reverse
float64 steer        # radians; positive turns left
```

`steer` is an angle, not a steering rate or steering-wheel rotation.
The header timestamp is not used to schedule commands: the latest received
command is applied at the next physics step. Speed and steering follow the
command through PI actuator loops, limits and noise; they do not change instantly.
Commands apply at any time, including before Start and after a run ends.
If no command arrives for `cmd_timeout` (default 0.5 s), the simulator requests
zero speed and holds the internal steering target. Braking still follows the
actuator dynamics; the car does not stop instantly.

`/veh_state` — actual rear-axle pose in the fixed `map` frame:

```text
std_msgs/Header header  # current ROS timestamp; frame_id = "map"
float64 x              # metres
float64 y              # metres
float64 yaw            # radians, wrapped to (-pi, pi]
```

At yaw 0 the car faces +x; positive yaw rotates toward +y (counterclockwise).
The default starting pose is `(0, 0, 0)`, configurable through
`init_x`, `init_y` and `init_yaw`. `/veh_state` does not contain speed or steer.

`/veh_actual` reuses `VehCtrl`, but `velocity` and `steer` contain **actual**
values after the PI loops and actuator noise. Its header matches `/veh_state`.
Read this topic for actual speed (m/s) and front-wheel steering angle (radians).
Do not publish commands on `/veh_actual`.

`/path` — ordered reference nodes and the stopping target:

```text
std_msgs/Header header       # frame_id = "map"
geometry_msgs/Point[] nodes  # x, y in metres; z = 0 (planar simulation)
uint32 finish_index         # zero-based index into nodes
```

- `nodes[0]` is the start; `nodes[finish_index]` is the finish/transition node.
- Nodes up to and including the finish form the segment to drive.
- Nodes from the finish onward form the run-out segment, allowing overshoot
  to be measured. Stop at the **finish node**, not the last node.
- An empty `nodes` array means **no task**: the driver should command zero
  velocity. Ignore `finish_index` for an empty path.
- For a circle the finish returns to the starting coordinates after one lap;
  track progress through the ordered nodes instead of testing position alone.

`/reset` has no fields. The vehicle resets its pose, actual speed/steering,
PI integrators, noise state and stored command. The driver should clear its
progress and internal controller state too. `/path` and `/reset` are separate
topics: do not assume their callbacks arrive in a particular order. A reset
published manually resets the vehicle; use the UI to reset evaluation state.

### Driver lifecycle

1. Start the driver; subscribe to `/path`, `/veh_state` and `/reset`.
   Subscribe to `/veh_actual` if your controller needs speed or steer feedback.
2. While there is no task, repeatedly publish a zero-speed `/veh_ctrl` command.
3. Click Start / Start evaluation. The simulator publishes the path and reset;
   follow the path using subsequent state messages and stop at the finish node.
4. Keep the driver alive across runs. Reset its progress for every new task/reset,
   and obey an empty path whenever one arrives (startup, Stop, evaluation end).

The simulator does not launch or pause your driver. Auto evaluation waits for a
`/path` subscriber; the UI connection dots monitor topic publishers. The physical
vehicle always responds to commands, so publishing an empty path is a request to
the driver to stop, not a direct brake or freeze of the vehicle.

### Evaluation result output

`/sim_status.data` contains `run`, `shape`, `result`, `score`, `tracking`,
`time_score`, `stop`, `v_eq`, `stop_err`, `completion`, `rms_cte`, `max_cte`,
and `time`.

- `score`: weighted total, 0–100, also published as `/score.data`.
- `tracking`, `time_score`, `stop`: component percentages, each 0–100;
  multiply by their weights (0.4, 0.4, 0.2) to obtain points shown in the UI.
- `v_eq`: equivalent speed in m/s; `time`: run duration in seconds.
- `completion`: fraction 0–1. `rms_cte`, `max_cte`: measured errors in metres.
- `stop_err`: signed distance along the path from finish to stop, in metres
  (negative before, positive after); JSON `null` when no stop position is recorded.
- `result`: `stopped`, `off_track`, `overrun`, `timeout`, or `aborted`.

### Inspect and publish from a terminal

After sourcing `install/setup.bash`, use a separate simulator session without
another driver publishing commands when trying the command example:

```bash
ros2 interface show thr_sim_msgs/msg/RefPath
ros2 topic echo /veh_state
ros2 topic echo /veh_actual
ros2 topic echo /path --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /sim_status

# Continuously command 2 m/s and 0.1 rad left steering (Ctrl-C to stop publishing).
ros2 topic pub --rate 20 /veh_ctrl thr_sim_msgs/msg/VehCtrl '{velocity: 2.0, steer: 0.1}'

# Reset the vehicle.
ros2 topic pub --once /reset std_msgs/msg/Empty '{}'
```

### Node overview

```
                /path (RefPath: nodes + finish_index, latched)
   sim_ctrl ───────────────────────────────────────► driver
      │  /reset (std_msgs/Empty)                       │
      ├────────────────────► vehicle_sim ◄─────────────┘
      │                        │        /veh_ctrl (VehCtrl: velocity, steer)
      ◄────────────────────────┴──────► driver
              /veh_state (VehState: x, y, yaw)
   sim_ctrl ──► /score (Float64), /sim_status (String, JSON)
   vehicle_sim ──► /veh_actual (VehCtrl: actual velocity, steer) ──► sim_ctrl (UI)
```

- **vehicle_sim** – always applies `/veh_ctrl` to the car; it never pauses.
  Integrates the kinematic bicycle model (`x' = v cos ψ`, `y' = v sin ψ`,
  `ψ' = v/L tan δ`, RK4, 100 Hz). Actual speed and steering follow the commands
  through PI loops on an integrator (`rate = kp·e + ki·∫e`, then
  `actual += rate·dt`): the speed loop outputs acceleration limited to
  `max_accel`/`max_decel`, the steering loop outputs steering rate limited to
  `max_steer_rate`; both use anti-windup (`kp = 2ζω`, `ki = ω²`). Colored
  Gaussian actuator noise (`speed_noise_std`, `steer_noise_std`, correlation
  time `noise_tau`) is added to the PI outputs; speed noise fades out below
  1 m/s; `noise_seed ≥ 0` gives the same noise every run, std 0 turns it off.
  Publishes `/veh_state` and `/veh_actual` at 50 Hz. `/reset` puts the vehicle
  back at its initial pose; without `/veh_ctrl` for `cmd_timeout` s it stops.
- **sim_ctrl** – generates the path, publishes `/path` and `/reset`, and scores
  each run (details below). Between runs and after Stop it publishes an empty
  path, meaning "no task".
- **driver** – demo P controller: steering = `kp_steer` × heading error to the
  path point a constant `lookahead` distance ahead; speed = constant `velocity`,
  0 once past the finish node. No YAML: the parameters are defaults in
  `driver_node.py` (`velocity` 10 m/s, `lookahead` 4.5 m, `kp_steer` 1.5),
  override with `--ros-args -p velocity:=5.0`.

## Paths and scoring

Each path (`straight`, `wave`, `lane_change`, `circle`) has two segments: the
segment to drive (`path_length` m in x, or one loop for `circle`) ends at the
**finish node**, followed by an **after-finish run-out** (`runout_length`,
default 20 m). The car must stop at the finish node; the run-out only absorbs
overshoot. `RefPath.finish_index` tells the driver which node is the finish.

A run ends with

| Result | When |
|---|---|
| `stopped` | the car has been at rest (moved < `stop_eps`) for `stop_hold` s after setting off |
| `off_track` | \|cross-track error\| > `max_cte` |
| `overrun` | the car reached the end of the run-out |
| `timeout` | `time_limit` exceeded |
| `aborted` | Stop / Reset / path change in the UI |

**Score (max 100) = tracking (40) + time (40) + stop (20)**
(`tracking_weight` 0.4, `time_weight` 0.4, stop gets the rest)

- tracking = 40 × completion × max(0, 1 − RMS cross-track error / `max_cte`),
  completion = progress up to the finish node
- time = 40 × min(v_eq, 30) / 30, linear in the equivalent speed
  v_eq = path length to the finish node / run time (start until the car came
  to rest); `time_v_max` = 30 m/s; 0 unless the run ended `stopped`
- stop = the full 20 when the car stops within `stop_free_zone` (5 m) of the
  finish node, then `stop_penalty` (5) points less per metre beyond that, so
  0 from 9 m; distance measured along the path, short or past; 0 unless the
  run ended `stopped`

Evaluate mode runs every shape in `eval_shapes` `num_runs` times, prints a
per-path summary (score, tracking, stop, mean |stop error|) and writes it with
every run's details to `results_file` (JSON).

## Sim control UI

The window uses a consistent light theme, a connection indicator for each node,
and a footer describing the plot signals. Live telemetry and run history are
in separate tabs; evaluation progress and the score summary stay visible above
them. Tables scroll horizontally and vertically to keep columns readable. The
scoring rules window scrolls on smaller displays.


`./run_sim.sh` opens a window (Tk + matplotlib):

- **Map**: path (dashed), after-finish run-out (dotted), red finish line,
  ±`max_cte` off-track corridor, vehicle with its actual steering angle, trail.
  Matplotlib toolbar to zoom/pan, *Follow vehicle*, *Fit path*.
- **CTE plot** and **control signals** (last 10 s): velocity and steer, solid =
  commanded (`/veh_ctrl`), dashed = actual (`/veh_actual`).
- **Controls**: path shape, runs, **Start** (runs on the selected path),
  **Evaluate** (every path × runs), **Stop**, **Reset** (keys: Enter, S, R).
  Picking a path aborts any run and resets the car.
- **Live / Results**: state, progress, distance to the finish, CTE, pose,
  command vs. actual, and a table of runs (path, result, track, time, stop,
  score – each as points / max – and stop error).
- **Evaluation window** (`evaluate.sh`): instead of the path / runs / Start
  controls the panel shows the evaluation plan, *▶ Start evaluation* (then
  *↻ Restart evaluation*, which discards a run in progress unscored) and *Stop*
  (key: S), a progress bar with the current run ("Running 3/12: wave (run
  1/3)"), and a per-path table coloured by score (≥ 80 green, ≥ 50 orange,
  < 50 red). Each part is shown as points earned / points possible, e.g.
  `Track 39.8/40`, `Time 2.2/40`, `Stop 18.9/20`, `Score 60.9/100`, plus runs
  done, runs stopped and mean |stop error|. Below it: `Overall score:
  60.9/100 = tracking 39.8/40 + time 2.2/40 + stop 18.9/20 (…, mean v_eq
  7.0 m/s)`. The per-run Results table uses the same fractions.
- **? Scoring rules** (in both panels): opens a window explaining how runs end
  and how tracking and stop points are computed, with a worked example; the
  numbers come from the current parameters, so it always matches the scoring.

Simulator parameters are in [simulator/thr_sim/config/sim.yaml](simulator/thr_sim/config/sim.yaml);
the demo driver's are defaults in [driver_node.py](thr_driver/thr_driver/driver_node.py).

## Manual build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch thr_sim sim.launch.py                         # simulator (what run_sim.sh runs)
ros2 launch thr_driver driver.launch.py                   # demo driver
ros2 launch thr_driver demo.launch.py                     # both in one launch
```

`sim.launch.py` arguments (also accepted by `demo.launch.py`): `path_shape`,
`num_runs`, `gui` (default true), `shutdown_on_finish`, `eval_mode`,
`eval_shapes`, `results_file`, `wait_for_driver` (default true: auto runs wait
for a `/path` subscriber).

Unit tests:

```bash
python3 -m pytest simulator/thr_sim/test thr_driver/test
```

## Writing your own driver

Use `thr_driver` as a template, or write a separate package that depends
only on `thr_sim_msgs`:

- subscribe to `/path` (`thr_sim_msgs/RefPath`, transient-local QoS) – the
  simulator treats a `/path` subscriber as "driver is up": follow
  `nodes`, stop at `nodes[finish_index]`; an empty path means stand still
- subscribe to `/veh_state` (`thr_sim_msgs/VehState`)
- publish `thr_sim_msgs/VehCtrl` on `/veh_ctrl`
- optionally listen to `/reset` to clear internal state between runs

Keep it running in the background and use `./run_sim.sh` / `./evaluate.sh`.
