# THRSimSimple

A ROS 2 Humble simulator for the freshman driverless-control assignment.
**Your job is to write a separate ROS 2 package that follows the reference path
and stops near its finish node.** The simulator supplies the path and vehicle
feedback; your node sends speed and steering commands.

## What you need to implement

Create your own Python `ament_python` package, for example `my_driver`, containing
a ROS 2 node that:

1. Receives the reference path from `/path`.
2. Receives the car's position and heading from `/veh_state`.
3. Calculates a speed command and a front-wheel steering angle.
4. Publishes those commands continuously on `/veh_ctrl` (20 Hz is a suitable starting rate).
5. Stops near `nodes[finish_index]`, handles an empty path, and resets its internal
   progress/controller state when a new run starts.

Implement the controller in your package. Communicate through ROS topics; do not
import the simulator's Python modules or change its messages, vehicle parameters,
paths or scoring to improve your score. An optional [demo_driver](demo_driver/README.md)
shows the simplest working example; use it to understand the topics, then implement
your own homework package. Run **only one driver at a time**, otherwise their
control commands compete.

You do not need to implement perception, vehicle physics, actuator PI loops or
the UI. A simple controller is enough to begin: first make straight-path driving
work, then add steering, progress tracking and stopping.

## First build and run

Run these commands in a terminal, from the workspace root:

```bash
cd ~/THRSimSimple
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source ROS and the workspace in **each new terminal** used for your driver or ROS
commands. Repeat the build after adding a package or changing its entry points.
See [Create your driver package](#create-your-driver-package) below before running
`my_driver` for the first time.

```bash
# Terminal 1: run YOUR driver and leave it running.
cd ~/THRSimSimple
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_driver driver
```

```bash
# Terminal 2: open the simulator, select a path, and click Start.
cd ~/THRSimSimple
./run_sim.bash

# After closing the simulator, open evaluation and click Start evaluation.
./evaluate.bash
```

Launch with `source ./run_sim.bash` or `source ./evaluate.bash`; direct execution
with `./` also works. When sourced, each launcher runs in a child Bash, so closing
the UI or a launch failure returns to your terminal without changing its directory,
environment or shell options. A sourced launcher prints failures and returns normally;
direct execution retains the launch command's exit status. Neither script launches a
driver. Your driver stays running across simulator runs and evaluation sessions.
Both terminals must use the same `ROS_DOMAIN_ID`. The simulator waits for a
driver. To watch the teaching example before implementing your own, run
`ros2 run demo_driver driver` in Terminal 1 instead. Stop it before starting your
own node.

Both scripts only source the environment and launch; build the workspace yourself
using the commands above before launching. Auto runs and
the evaluation wait for vehicle feedback and a driver subscribed to `/path`
and publishing `/veh_ctrl`, logging every 5 s
while they wait; the driver can keep running across many simulator sessions.

| Script | Options |
|---|---|
| `run_sim.bash` | `--headless`, plus launch args such as `path_shape:=circle num_runs:=3` |
| `evaluate.bash` | `-n RUNS` (default 3), `-s SHAPES` (default all), `-o FILE` (default `eval_results/eval_<date>_<time>.json`), `--headless` (no UI, exits when done) |

## Evaluation: Set up → Run tests → Review

Start your driver in one terminal and run `./evaluate.bash` in another. The
scripts only launch the simulator; build your package first.

1. **Set up:** choose the paths and repeats per path in the evaluation window.
   The plan shows the total number of tests. Connection checks show whether the
   vehicle is publishing feedback and the driver is subscribed to `/path` and
   publishing `/veh_ctrl`. No evaluation starts until you click **Start evaluation**.
2. **Run tests:** clicking Start captures and locks the plan. If connections
   are missing, it waits; **Cancel evaluation** also cancels that wait. Each test
   sends a new path and reset. The UI shows the path, repeat, progress and latest
   result, followed by a short countdown (default 2 s) before the next test.
   During this transition an empty path asks your driver to stop. Physics and
   live plots keep updating; the transition does not count toward the time score.
3. **Review:** after all tests, the simulator sends an empty path and selects
   the **Path scores** tab. Review the tracking/time/stopping scorecards, per-path
   averages and individual **Run results**. Outcome hints explain common failures;
   the saved JSON report path is shown in the review area. No graph popup opens.

**Cancel evaluation** stops scheduling tests, asks the driver to stop, and
preserves only finished tests. The current unfinished test is discarded without
adding a zero/aborted score. Partial results are saved if at least one test
finished. The UI and report explicitly identify partial results and how many
planned tests were completed. Their average is not a full-plan evaluation score.

Use **Run again** after completion or cancellation to edit the plan and start a
fresh evaluation. Results clear when the new evaluation is requested; completed
results remain visible until then. The current `results_file` is reused when
running again, so copy reports you want to keep before rerunning in that window.

Reports go to `eval_results/eval_<date>_<time>.json` plus a PNG chart. JSON includes
`status` (`complete` or `cancelled`), `plan`, `planned_runs`, `completed_runs`,
`overall_score`, `per_path` and the individual `runs`. The score rules remain
tracking 40 + time 40 + stopping 20. **Scoring rules** explains the current values.
The window stays open after completion. `--headless` uses the command-line plan,
starts automatically when connections are ready, and exits when done.

## Packages

| Package | Contents | Depends on |
|---|---|---|
| `simulator/thr_sim_msgs` | `VehCtrl` (velocity, steer), `VehState` (x, y, yaw), `RefPath` (nodes, finish index) | – |
| `simulator/thr_sim` | simulator: `vehicle_sim`, `sim_ctrl` (+ UI), `sim.launch.py`, `sim.yaml` | `thr_sim_msgs` |
| `demo_driver` | optional teaching example: fixed lookahead, P steering, constant low speed | `rclpy`, `std_msgs`, `thr_sim_msgs` |

The simulator and the driver only talk through ROS topics; neither imports
the other's code.

## Simulator I/O (ROS 2 topics)


The simulator consists of `vehicle_sim` (vehicle physics) and `sim_ctrl`
(paths, resets, evaluation and UI). The external driver runs separately on the
same `ROS_DOMAIN_ID`. Directions below are **relative to the simulator**:
input means the driver/user publishes it; output means the simulator publishes it.

### Minimum interface for YOUR driver

| In your node | Topic | Python message import | What to do |
|---|---|---|---|
| Subscribe | `/path` | `from thr_sim_msgs.msg import RefPath` | Store ordered nodes and `finish_index`; an empty path means stop |
| Subscribe | `/veh_state` | `from thr_sim_msgs.msg import VehState` | Read `x`, `y`, `yaw` for steering and progress |
| Subscribe | `/reset` | `from std_msgs.msg import Empty` | Clear progress and controller memory for the next run |
| Publish | `/veh_ctrl` | `from thr_sim_msgs.msg import VehCtrl` | Send `velocity` in m/s and `steer` in radians continuously |
| Subscribe (optional) | `/veh_actual` | `from thr_sim_msgs.msg import VehCtrl` | Read actual velocity/steering for feedback |

```text
Simulator  -- /path, /veh_state, /veh_actual, /reset -->  Your driver
Simulator  <--------------- /veh_ctrl ----------------  Your driver
```

Start with these five topics. `/score` and `/sim_status` report results; your
controller does not need them to drive. The detailed reference below lists every
topic from the simulator's point of view.

### Topic reference

| Direction | Topic | ROS message type | Producer → consumer | When / rate |
|---|---|---|---|---|
| Input | `/veh_ctrl` | `thr_sim_msgs/msg/VehCtrl` | Driver → `vehicle_sim` (also displayed by `sim_ctrl`) | Publish continuously; 20 Hz suggested |
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
`/path` subscriber and `/veh_ctrl` publisher; the UI connection dots monitor topic publishers. The physical
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
- **Your driver** – a student-written ROS 2 node in a separate package.
  It receives the path and vehicle feedback, computes commands and publishes
  `/veh_ctrl`. The communication skeleton below contains no control algorithm.

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
per-path summary (score, tracking, time, stop, mean |stop error|) and writes it with
every run's details to `results_file` (JSON).

## Sim control UI

The Driverless Lab window has a resizable split between the map/plots and the
control panel: drag the divider to give either side more room. Four live cards
across the top show actual speed, actual steering, path error and elapsed run
time, even while you inspect results. The initial window size adapts to the
screen. Vehicle and driver connection indicators show whether the nodes are up.

The map sits above three separate plots: cross-track error, speed and steering.
Each signal plot keeps a fixed 10-second window; solid lines are commands and
dashed lines are actual values. The interface continues plotting while idle.

In normal mode the panel has **Telemetry** and **Run results** tabs. Evaluation
adds a **Path scores** tab and persistent scorecards for tracking, finish time
and stopping; progress and the overall score remain above the tabs. Tables
scroll horizontally and vertically. The scoring rules window is scrollable.

`./run_sim.bash` opens a window (Tk + matplotlib):

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
- **Evaluation window** (`evaluate.bash`): choose paths and repeats, then click
  **Start evaluation**. The Set up / Run tests / Review stages show readiness,
  the active test, between-test countdowns and completion. **Cancel evaluation**
  (key: S) keeps finished tests; **Run again** starts a fresh plan. The **Path
  scores** table is coloured by score (≥ 80 green, ≥ 50 orange, < 50 red). Each part is shown as points earned / points possible, e.g.
  `Track 39.8/40`, `Time 2.2/40`, `Stop 18.9/20`, `Score 60.9/100`, plus runs
  done, runs stopped and mean |stop error|. Below it: `Overall score:
  60.9/100 = tracking 39.8/40 + time 2.2/40 + stop 18.9/20 (…, mean v_eq
  7.0 m/s)`. The per-run Results table uses the same fractions.
- **? Scoring rules** (in both panels): opens a window explaining how runs end
  and how tracking and stop points are computed, with a worked example; the
  numbers come from the current parameters, so it always matches the scoring.

Simulator parameters are in [simulator/thr_sim/config/sim.yaml](simulator/thr_sim/config/sim.yaml).
Keep your controller parameters in your own package.

## Manual build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch thr_sim sim.launch.py                         # simulator (what run_sim.bash runs)
ros2 run my_driver driver                                # your implementation, in another terminal
```

`sim.launch.py` arguments: `path_shape`,
`num_runs`, `gui` (default true), `shutdown_on_finish`, `eval_mode`,
`eval_shapes`, `results_file`, `wait_for_driver` (default true: auto runs wait
for a `/path` subscriber and `/veh_ctrl` publisher).

Unit tests:

```bash
python3 -m pytest simulator/thr_sim/test demo_driver/test
```

## Optional teaching example

[demo_driver](demo_driver/README.md) is a deliberately simple example, separate
from the simulator and the communication-only skeleton below. Its controller is
one function in [driver_node.py](demo_driver/demo_driver/driver_node.py): select a
node 4 m ahead, steer proportionally toward it, and drive at 3 m/s until reaching
the finish index. It requests zero for an empty path or missing vehicle feedback.
There is no YAML, speed controller or simulator-code dependency.

Read its callbacks to understand `/path`, `/veh_state`, `/reset` and `/veh_ctrl`.
It is a baseline, not an optimal scoring solution: it does not plan braking and
uses discrete path nodes. The simulator scripts still launch only the simulator;
run the example separately and stop it before testing your homework node.

## Create your driver package

Create your package next to `simulator/` and `demo_driver/`, at the workspace root. Colcon discovers
packages recursively; this workspace does not require a `src/` directory.

```bash
cd ~/THRSimSimple
source /opt/ros/humble/setup.bash
ros2 pkg create my_driver --build-type ament_python \
  --dependencies rclpy std_msgs thr_sim_msgs
```

Create `my_driver/my_driver/driver_node.py`. The following skeleton sets up the
communication only. **It commands zero speed until you implement the TODO.**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty
from thr_sim_msgs.msg import RefPath, VehCtrl, VehState


class Driver(Node):
    def __init__(self):
        super().__init__('my_driver')
        self.nodes = []
        self.finish_index = 0
        self.state = None
        self.progress_index = 0

        # /path is retained: receive the latest task even if connecting late.
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(RefPath, '/path', self.on_path, path_qos)
        self.create_subscription(VehState, '/veh_state', self.on_state, 10)
        self.create_subscription(Empty, '/reset', self.on_reset, 10)
        self.publisher = self.create_publisher(VehCtrl, '/veh_ctrl', 10)
        self.create_timer(0.05, self.control)  # 20 Hz

    def on_path(self, msg):
        self.nodes = list(msg.nodes)
        self.finish_index = msg.finish_index
        self.progress_index = 0
        # TODO: also clear any controller memory for this new task.

    def on_state(self, msg):
        self.state = msg

    def on_reset(self, _msg):
        self.state = None  # wait for fresh vehicle feedback after the reset
        self.progress_index = 0
        # Keep the current path: /path and /reset may arrive in either order.
        # TODO: clear integrators, filters and other controller memory.

    def control(self):
        velocity, steer = 0.0, 0.0
        if (len(self.nodes) >= 2 and self.state is not None
                and 0 <= self.finish_index < len(self.nodes)):
            # TODO: calculate velocity and steer using the path and state.
            # The stopping target is self.nodes[self.finish_index].
            pass

        # Continue publishing zero while waiting or when the task is empty.
        msg = VehCtrl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = float(velocity)
        msg.steer = float(steer)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

In the generated `my_driver/setup.py`, add this entry to `console_scripts`:

```python
entry_points={
    'console_scripts': [
        'driver = my_driver.driver_node:main',
    ],
},
```

Keep the other generated setup fields. Build and source the workspace again:

```bash
cd ~/THRSimSimple
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run my_driver driver
```

### Control behavior to think about

- **Steering:** choose a target ahead on the path, compare its direction with
  `yaw`, and calculate a steering angle. Angles are radians; wrap heading errors
  to avoid jumps at ±pi. Positive steering turns left.
- **Progress:** follow the node order. On the circle the start and finish share
  coordinates, so distance to the finish alone cannot tell you whether a lap is complete.
- **Stopping:** setting `velocity = 0` requests braking; the vehicle still needs
  time and distance to stop. Consider slowing down before the finish.
- **Actual vs commanded:** actuator limits, PI dynamics and noise mean the car
  does not immediately reach your requested speed or steering angle.
- **Between tasks:** an empty path means stop. Do not use the last path forever
  or exit the node after a single run.

### Test your implementation

1. Run your node with the skeleton first: it should appear connected in the UI
   and hold zero speed before Start.
2. Add your controller and test `straight` at a low speed.
3. Test `wave`, `lane_change` and `circle`; watch the trail and CTE plot.
4. Check that the car stops near the red finish line, rather than at the end
   of the dotted run-out.
5. Click Reset and start again. Check new paths, repeated runs, Cancel evaluation, and Run again
   without restarting your node.
6. Run `./evaluate.bash`, click **Start evaluation**, and inspect the per-part
   scores and `eval_results/` files. Keep the provided simulator settings for
   comparisons between team members.

A useful submission includes your package, its run command and dependencies,
a short explanation of your controller and chosen parameters, and an evaluation
result with notes on any failing scenarios. Follow any additional instructions
from your team lead.

## Troubleshooting

| Symptom | What to check |
|---|---|
| `ament_cmake` cannot be found | Run `source /opt/ros/humble/setup.bash` before building |
| `Package 'my_driver' not found` | Build the package, then source `install/setup.bash` in that terminal |
| Driver connects but gets no path | Subscribe using `RefPath` and the transient-local QoS shown above; click Start |
| Evaluation keeps waiting for a driver | Check your `/path` subscription, matching `ROS_DOMAIN_ID`, and sourced workspace |
| Car moves before Start or after Stop | Handle empty paths and publish zero velocity while no task is active |
| Car briefly drives then slows unexpectedly | Publish commands continuously; `cmd_timeout` defaults to 0.5 s |
| Car turns the wrong way | Check steering sign and radians vs degrees |
| Car stops far past the red line | Zero speed is a command to the actuator; allow for braking distance |
| Circle appears finished immediately | Track ordered path progress; the finish is at the start coordinates after one lap |
| Motion is inconsistent | Ensure only one driver is publishing `/veh_ctrl`; inspect `ros2 topic info /veh_ctrl --verbose` |
| Terminal closes when the UI closes | Use `source ./run_sim.bash` or `source ./evaluate.bash` from an open terminal. These launch in a child Bash. A terminal task configured solely for the launcher may still close when the task ends |
| Launcher exits with code 1 | Run `./run_sim.bash` in an open terminal and read the first error; both `source` and direct execution are supported |

Useful checks, in a terminal with the workspace sourced:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /path --verbose
ros2 topic info /veh_ctrl --verbose
ros2 topic echo /veh_state --once
ros2 topic echo /veh_actual --once
```
