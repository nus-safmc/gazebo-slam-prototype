#!/usr/bin/env bash
set -euo pipefail

ARGS=()

# Pixi commonly uses `--` to separate task args; tolerate it.
while [[ $# -gt 0 ]]; do
  a="$1"
  shift
  if [[ "$a" == "--" ]]; then
    continue
  fi
  ARGS+=("$a")
done

bash "$(dirname "$0")/cleanup_sim.sh"
bash "$(dirname "$0")/check_microxrce_agent.sh"

exec ros2 launch tof_slam_sim px4_nav2_explore.launch.py "${ARGS[@]}"

