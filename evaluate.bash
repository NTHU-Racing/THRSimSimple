#!/usr/bin/env bash
# Auto-evaluate the driver that is running in the background: run every path
# shape N times, print a summary and write it to a JSON file plus a PNG chart.
# In the UI press "Start evaluation"; the window stays open afterwards with the
# results (close it to exit). --headless starts by itself and exits when done.
# The driver is NOT started here: run it yourself first (same ROS_DOMAIN_ID);
# evaluation waits for its /path subscription and /veh_ctrl publisher.
# Driver example command (after implementing it): ros2 run my_driver driver
#
# Usage: source ./evaluate.bash [-n RUNS] [-s SHAPES] [-o FILE] [--headless] [launch args...]
#   -n RUNS       runs per path shape (default 3)
#   -s SHAPES     comma-separated shapes (default straight,wave,lane_change,circle)
#   -o FILE       results file (default eval_results/eval_<date>_<time>.json)
#   --headless    no UI; exit as soon as the evaluation is done
# When sourced, run in a child Bash so setup, exit and shell options stay local.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  if bash "${BASH_SOURCE[0]}" "$@"; then
    return 0
  else
    printf 'Launcher exited with status %s; your terminal is still available.\n' "$?" >&2
    return 0
  fi
fi

set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNS=3
SHAPES="straight,wave,lane_change,circle"
OUT="$WS/eval_results/eval_$(date +%Y%m%d_%H%M%S).json"
GUI=1
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n) RUNS="$2"; shift ;;
    -s) SHAPES="$2"; shift ;;
    -o) OUT="$(realpath -m "$2")"; shift ;;
    --headless)  GUI=0 ;;
    -h|--help)   sed -n '2,/^# When sourced/{ /^# When sourced/d; p; }' "$0"; exit 0 ;;
    *)           ARGS+=("$1") ;;
  esac
  shift
done

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$WS"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace is not built. Source ROS, run colcon build --symlink-install, then launch again." >&2
  exit 1
fi
source "$WS/install/setup.bash"

# defaults first, so launch args given on the command line override them
if [ "$GUI" = 1 ]; then
  ARGS=("gui:=true" "shutdown_on_finish:=false" "${ARGS[@]}")
  echo "Press 'Start evaluation' in the window; it stays open afterwards (close it to exit)."
else
  ARGS=("gui:=false" "shutdown_on_finish:=true" "${ARGS[@]}")
fi

echo "Evaluating ${SHAPES} x ${RUNS} run(s); driver expected on ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
ros2 launch thr_sim sim.launch.py eval_mode:=true num_runs:="$RUNS" eval_shapes:="$SHAPES" \
  results_file:="$OUT" "${ARGS[@]}"

