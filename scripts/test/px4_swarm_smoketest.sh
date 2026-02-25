#!/usr/bin/env bash
set -euo pipefail

# PX4 uXRCE-DDS endpoints are not reliably discovered via CycloneDDS in this stack.
# Force FastDDS for all PX4 runs.
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
unset CYCLONEDDS_URI

usage() {
  cat >&2 <<'EOF'
usage:
  bash scripts/px4_swarm_smoketest.sh [--num-drones N] [--startup-wait-sec N] [--motion-sec N] [--motion-min-m X] [--headless-rendering true|false]

Starts PX4 swarm_fast (no RViz / no Gazebo GUI), then verifies:
  - /depth/tof_1 publishes
  - /scan_merged publishes
  - /map is published (and not pre-sized to the full arena)
  - at least a majority of /<robot>/odom topics show motion over time

Examples:
  pixi run -e jazzy bash scripts/px4_swarm_smoketest.sh
  pixi run -e jazzy bash scripts/px4_swarm_smoketest.sh --num-drones 5 --headless-rendering true
EOF
}

NUM_DRONES="5"
STARTUP_WAIT_SEC="18"
MOTION_SEC="10"
MOTION_MIN_M="0.25"
HEADLESS_RENDERING="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --num-drones|--num|--n) NUM_DRONES="${2:-}"; shift 2 ;;
    --startup-wait-sec) STARTUP_WAIT_SEC="${2:-}"; shift 2 ;;
    --motion-sec) MOTION_SEC="${2:-}"; shift 2 ;;
    --motion-min-m) MOTION_MIN_M="${2:-}"; shift 2 ;;
    --headless-rendering) HEADLESS_RENDERING="${2:-}"; shift 2 ;;
    *) echo "[swarm_smoketest] Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[swarm_smoketest] ros2 is not on PATH. Run via pixi:" >&2
  echo "  pixi run -e jazzy bash scripts/px4_swarm_smoketest.sh" >&2
  exit 127
fi

if ! [[ "$NUM_DRONES" =~ ^[0-9]+$ ]]; then
  echo "[swarm_smoketest] error: --num-drones expects an integer (got '$NUM_DRONES')" >&2
  exit 2
fi

bash "$(dirname "$0")/cleanup_sim.sh"
bash "$(dirname "$0")/check_microxrce_agent.sh"

set +e
ros2 launch tof_slam_sim px4_swarm_fast.launch.py \
  default_spawn:=true \
  num_robots:="$NUM_DRONES" \
  rviz:=false \
  gz_gui:=false \
  headless_rendering:="$HEADLESS_RENDERING" &
LAUNCH_PID=$!
set -e

cleanup() {
  echo "[swarm_smoketest] Cleaning up..."
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  bash "$(dirname "$0")/cleanup_sim.sh" || true
}
trap cleanup EXIT

echo "[swarm_smoketest] Waiting for startup (${STARTUP_WAIT_SEC}s)..."
sleep "${STARTUP_WAIT_SEC}"

echo "[swarm_smoketest] Checking ToF bridge (/depth/tof_1)..."
ros2 topic echo --once --timeout 10 /depth/tof_1 sensor_msgs/msg/Image >/dev/null

echo "[swarm_smoketest] Checking scan pipeline (/scan_merged)..."
ros2 topic echo --once --timeout 10 /scan_merged sensor_msgs/msg/LaserScan >/dev/null

echo "[swarm_smoketest] Checking map publication (/map)..."
MAP_INFO="$(python3 scripts/inspect_map_once.py --topic /map --timeout-sec 10.0 --border-cells 5)"
echo "${MAP_INFO}"

W="$(printf '%s\n' "${MAP_INFO}" | sed -n 's/.* w=\([0-9][0-9]*\) .*/\1/p' | head -n 1)"
H="$(printf '%s\n' "${MAP_INFO}" | sed -n 's/.* h=\([0-9][0-9]*\) .*/\1/p' | head -n 1)"
if [[ -n "$W" && -n "$H" ]]; then
  # If we accidentally seed the full arena, the map is ~800x800 at 0.05m for the 40x40 playfield.
  # With multiple drones, the fuser can expand early to cover all spawn points + ToF range, so allow
  # moderate sizes as long as we don't start at the full arena bounds.
  if [[ "$W" -gt 700 || "$H" -gt 700 ]]; then
    echo "[swarm_smoketest] ERROR: map appears pre-sized (w=${W} h=${H})." >&2
    exit 2
  fi
fi

echo "[swarm_smoketest] Checking motion (/odom, /robot2/odom...)..."

REQUIRED_MOVERS=$(( (NUM_DRONES / 2) + 1 ))
MOVED=0

topics=(/odom)
if [[ "$NUM_DRONES" -gt 1 ]]; then
  for i in $(seq 2 "$NUM_DRONES"); do
    topics+=("/robot${i}/odom")
  done
fi

for t in "${topics[@]}"; do
  MOTION_OUT="$(python3 scripts/check_motion.py "$t" --duration-sec "${MOTION_SEC}")" || {
    echo "[swarm_smoketest] ERROR: no odom samples on $t" >&2
    exit 2
  }
  echo "${MOTION_OUT}"

  # Prefer max displacement from the start so we still detect movement even if the robot returns.
  MAX_DIST="$(printf '%s\n' "${MOTION_OUT}" | sed -n 's/.*max_displacement=\([0-9.][0-9.]*\)m.*/\1/p' | head -n 1)"
  if [[ -z "${MAX_DIST}" ]]; then
    echo "[swarm_smoketest] ERROR: failed to parse max_displacement from: ${MOTION_OUT}" >&2
    exit 2
  fi
  if awk -v a="${MAX_DIST}" -v b="${MOTION_MIN_M}" 'BEGIN{exit !(a>=b)}'; then
    MOVED=$((MOVED + 1))
  fi
done

if [[ "${MOVED}" -lt "${REQUIRED_MOVERS}" ]]; then
  echo "[swarm_smoketest] ERROR: only ${MOVED}/${NUM_DRONES} drones moved >= ${MOTION_MIN_M}m (need ${REQUIRED_MOVERS})." >&2
  exit 2
fi
echo "[swarm_smoketest] OK: ${MOVED}/${NUM_DRONES} drones moved >= ${MOTION_MIN_M}m (need ${REQUIRED_MOVERS})."

echo "[swarm_smoketest] OK"
