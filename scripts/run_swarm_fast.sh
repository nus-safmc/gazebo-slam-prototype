#!/usr/bin/env bash
set -euo pipefail

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
  if [[ "$a" == "--n" || "$a" == "--num" ]]; then
    if [[ $# -lt 1 ]]; then
      echo "error: $a requires an integer" >&2
      exit 2
    fi
    N_ROBOTS="$1"
    shift
    continue
  fi
  if [[ "$a" == "--num-drones" ]]; then
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

bash "$(dirname "$0")/cleanup_sim.sh"

ROBOTS_ARG=()
if [[ -n "$ROBOTS_CSV" ]]; then
  ROBOTS_ARG=("robots:=$ROBOTS_CSV")
elif [[ -n "$N_ROBOTS" ]]; then
  if ! [[ "$N_ROBOTS" =~ ^[0-9]+$ ]]; then
    echo "error: --n expects an integer (got '$N_ROBOTS')" >&2
    exit 2
  fi
  if [[ "$N_ROBOTS" -lt 1 ]]; then
    echo "error: --n must be >= 1" >&2
    exit 2
  fi
  if [[ "$N_ROBOTS" -gt 15 ]]; then
    echo "error: --n must be <= 15 (got '$N_ROBOTS')" >&2
    exit 2
  fi
  ROBOTS_ARG=("num_robots:=$N_ROBOTS")
fi

if [[ "$DEFAULT_SPAWN" -eq 1 ]]; then
  exec ros2 launch tof_slam_sim swarm_fast.launch.py default_spawn:=true "${ROBOTS_ARG[@]}" "${ARGS[@]}"
else
  exec ros2 launch tof_slam_sim swarm_fast.launch.py default_spawn:=false "${ROBOTS_ARG[@]}" "${ARGS[@]}"
fi
