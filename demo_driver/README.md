# Demo driver: the smallest useful controller

This teaching example shows how to connect a student ROS 2 package to the
simulator. The complete node and control calculation are in
[driver_node.py](demo_driver/driver_node.py). It uses only Python's `math`, ROS 2
and the public messages; it does not import simulator code.

## Run it

Build from the workspace root:

```bash
cd ~/THRSimSimple
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run demo_driver driver
```

Leave the driver running. In another terminal:

```bash
cd ~/THRSimSimple
source ./run_sim.bash
# Or, after closing the simulator:
source ./evaluate.bash
```

Click Start / Start evaluation in the UI. The scripts do not launch this node.
Stop other drivers before running the example: only one node should publish
commands to `/veh_ctrl`.

## The method

Every 0.05 seconds (20 Hz):

1. Find the nearest path node in a small forward window. Keep the index moving
   forward, so the circle does not jump to its finish while still at the start.
2. Walk 4 m along the path from that node to select a target.
3. Calculate the direction to the target and subtract the car's yaw.
4. Wrap that heading error to ±pi and command `steer = gain * error`, limited
   to ±0.6 radians. Positive steering turns left.
5. Command a constant 3 m/s until the progress index reaches the finish node;
   then command zero speed and zero steer.

This is proportional heading control, not a complete vehicle controller. The
low speed makes it a simple baseline. It has no speed planning or braking-distance
calculation: the car keeps moving while its actual speed falls after a zero
command, so it may stop past the finish. It uses discrete nodes rather than
projecting onto/interpolating path segments. Students can improve these parts
in their own packages.

## The ROS callbacks

| Callback / timer | Topic | Job |
|---|---|---|
| `on_path` | `/path` (`RefPath`, transient-local QoS) | Store nodes and finish index; clear progress; empty or invalid path means no task |
| `on_state` | `/veh_state` (`VehState`) | Store the latest `(x, y, yaw)` |
| `on_reset` | `/reset` (`Empty`) | Clear progress and pose; keep the path because reset/path callbacks can arrive in either order |
| `control` | publishes `/veh_ctrl` (`VehCtrl`) | Calculate and publish commands at 20 Hz; publish zero while waiting |

The controller does not need `/veh_actual`, `/score` or `/sim_status`.
`finish_index` marks the stopping target; the last path node is the end of the
run-out, not the finish.

## Parameters (defaults in the Python file; no YAML)

| Parameter | Default | Meaning |
|---|---|---|
| `speed` | 3.0 | Constant commanded speed in m/s |
| `lookahead` | 4.0 | Distance forward along the nodes in metres |
| `gain` | 1.0 | Steering angle per radian of heading error |

For example:

```bash
ros2 run demo_driver driver --ros-args -p speed:=2.0
```

A higher speed is not guaranteed to work with this trivial controller.
