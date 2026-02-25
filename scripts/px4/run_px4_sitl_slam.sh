#!/usr/bin/env bash
set -euo pipefail

# PX4 uXRCE-DDS endpoints are not reliably discovered via CycloneDDS in this stack.
# Force FastDDS for all PX4 runs.
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
unset CYCLONEDDS_URI

ARGS=()
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

exec ros2 launch tof_slam_sim px4_sitl_slam.launch.py "${ARGS[@]}"
