#!/usr/bin/env bash
set -euo pipefail

# Keep the pixi-env default DDS (CycloneDDS).  FastDDS hangs in RoboStack
# environments.  PX4 uXRCE-DDS topics are still discoverable via standard
# DDS interop even when the ROS side uses CycloneDDS.

ARGS=()
while [[ $# -gt 0 ]]; do
  a="$1"
  shift
  if [[ "$a" == "--" ]]; then
    continue
  fi
  ARGS+=("$a")
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
bash "$SCRIPT_DIR/../core/cleanup_sim.sh"
bash "$SCRIPT_DIR/check_microxrce_agent.sh"

exec ros2 launch tof_slam_sim px4_sitl_slam.launch.py ${ARGS[@]+"${ARGS[@]}"}
