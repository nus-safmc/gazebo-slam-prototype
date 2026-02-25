#!/usr/bin/env bash
set -euo pipefail

# Keep the pixi-env default DDS (CycloneDDS).  FastDDS hangs in RoboStack
# environments.  PX4 uXRCE-DDS topics are still discoverable via standard
# DDS interop even when the ROS side uses CycloneDDS.

DEFAULT_SPAWN=0
N_ROBOTS=""
ROBOTS_CSV=""
ARGS=()

# Pixi commonly uses `--` to separate task args; tolerate it either way.
while [[ $# -gt 0 ]]; do
  a="$1"
  shift
  if [[ "$a" == "--" ]]; then
    continue
  fi
  if [[ "$a" == "--default" ]]; then
    DEFAULT_SPAWN=1
    continue
  fi
  if [[ "$a" == "--n" || "$a" == "--num" || "$a" == "--num-drones" ]]; then
    if [[ $# -lt 1 ]]; then
      echo "error: $a requires an integer" >&2
      exit 2
    fi
    N_ROBOTS="$1"
    shift
    continue
  fi
  if [[ "$a" == "--robots" ]]; then
    if [[ $# -lt 1 ]]; then
      echo "error: --robots requires a comma-separated list" >&2
      exit 2
    fi
    ROBOTS_CSV="$1"
    shift
    continue
  fi
  ARGS+=("$a")
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
bash "$SCRIPT_DIR/../core/cleanup_sim.sh"
bash "$SCRIPT_DIR/../px4/check_microxrce_agent.sh"

ROBOTS_ARG=()
if [[ -n "$ROBOTS_CSV" ]]; then
  ROBOTS_ARG=("robots:=$ROBOTS_CSV")
elif [[ -n "$N_ROBOTS" ]]; then
  if ! [[ "$N_ROBOTS" =~ ^[0-9]+$ ]]; then
    echo "error: --num-drones expects an integer (got '$N_ROBOTS')" >&2
    exit 2
  fi
  if [[ "$N_ROBOTS" -lt 1 ]]; then
    echo "error: --num-drones must be >= 1" >&2
    exit 2
  fi
  if [[ "$N_ROBOTS" -gt 15 ]]; then
    echo "error: --num-drones must be <= 15 (got '$N_ROBOTS')" >&2
    exit 2
  fi
  ROBOTS_ARG=("num_robots:=$N_ROBOTS")
fi

if [[ "$DEFAULT_SPAWN" -eq 1 ]]; then
  exec ros2 launch tof_slam_sim px4_swarm_fast.launch.py default_spawn:=true ${ROBOTS_ARG[@]+"${ROBOTS_ARG[@]}"} ${ARGS[@]+"${ARGS[@]}"}
else
  exec ros2 launch tof_slam_sim px4_swarm_fast.launch.py default_spawn:=false ${ROBOTS_ARG[@]+"${ROBOTS_ARG[@]}"} ${ARGS[@]+"${ARGS[@]}"}
fi
