#!/usr/bin/env bash
# Launch the simulator (vehicle_sim + sim_ctrl with the UI).
# The driver is NOT started here: run it in the background yourself (same
# ROS_DOMAIN_ID); it must react to /path, /veh_state (and optionally /reset)
# and publish /veh_ctrl. Demo driver: ros2 launch thr_driver driver.launch.py
#
# Usage: ./run_sim.sh [--headless] [--no-build] [launch args...]
#   --headless    no UI; runs start once a driver subscribes to /path
#   --no-build    skip `colcon build`
#   launch args   passed to sim.launch.py, e.g. path_shape:=circle num_runs:=3
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD=1
ARGS=()
for a in "$@"; do
  case "$a" in
    --headless)  ARGS+=("gui:=false") ;;
    --no-build)  BUILD=0 ;;
    -h|--help)   sed -n '2,10p' "$0"; exit 0 ;;
    *)           ARGS+=("$a") ;;
  esac
done

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$WS"
if [ "$BUILD" = 1 ]; then
  log="$(mktemp)"
  echo "Building workspace..."
  colcon build --symlink-install >"$log" 2>&1 || { cat "$log"; echo "Build failed."; exit 1; }
  rm -f "$log"
fi
source "$WS/install/setup.bash"

exec ros2 launch thr_sim sim.launch.py "${ARGS[@]}"
