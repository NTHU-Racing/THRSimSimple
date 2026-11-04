#!/usr/bin/env bash
# Launch the simulator (vehicle_sim + sim_ctrl with the UI).
# The driver is NOT started here: run it in the background yourself (same
# ROS_DOMAIN_ID); it must react to /path, /veh_state (and optionally /reset)
# and publish /veh_ctrl. Students implement the driver in their own package.
#
# Usage: source ./run_sim.bash [--headless] [launch args...]
#   --headless    no UI; runs start once a driver subscribes to /path
#   launch args   passed to sim.launch.py, e.g. path_shape:=circle num_runs:=3
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
ARGS=()
for a in "$@"; do
  case "$a" in
    --headless)  ARGS+=("gui:=false") ;;
    -h|--help)   sed -n '2,/^# When sourced/{ /^# When sourced/d; p; }' "$0"; exit 0 ;;
    *)           ARGS+=("$a") ;;
  esac
done

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$WS"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace is not built. Source ROS, run colcon build --symlink-install, then launch again." >&2
  exit 1
fi
source "$WS/install/setup.bash"

ros2 launch thr_sim sim.launch.py "${ARGS[@]}"
