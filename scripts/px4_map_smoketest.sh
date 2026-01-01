#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  bash scripts/px4_map_smoketest.sh [--world <file.sdf>] [--duration-sec N] [--out <maps/name>] [--slam-config <yaml>]

Runs PX4+Nav2 exploration for N seconds, then saves a map snapshot:
  <out>.pgm / <out>.yaml / <out>.png / <out>_x4.png

Examples:
  pixi run -e jazzy bash scripts/px4_map_smoketest.sh
  pixi run -e jazzy bash scripts/px4_map_smoketest.sh --world playfield_px4_sparse.sdf --duration-sec 240 --out maps/px4_sparse_4min

Tip (logs):
  FILTER_RMW_TYPEHASH=1 bash scripts/run_with_log.sh px4_map_smoketest bash scripts/px4_map_smoketest.sh
EOF
}

WORLD="playfield_px4_small.sdf"
DURATION_SEC="180"
OUT_BASE="maps/px4_map_smoketest"
SLAM_CONFIG="slam_toolbox_px4_robust.yaml"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --world) WORLD="${2:-}"; shift 2 ;;
    --duration-sec) DURATION_SEC="${2:-}"; shift 2 ;;
    --out) OUT_BASE="${2:-}"; shift 2 ;;
    --slam-config) SLAM_CONFIG="${2:-}"; shift 2 ;;
    *) echo "[smoketest] Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[smoketest] ros2 is not on PATH. Run via pixi:" >&2
  echo "  pixi run -e jazzy bash scripts/px4_map_smoketest.sh" >&2
  exit 127
fi

OUT_DIR="$(dirname "$OUT_BASE")"
mkdir -p "$OUT_DIR"

echo "[smoketest] world: ${WORLD}"
echo "[smoketest] duration: ${DURATION_SEC}s"
echo "[smoketest] out: ${OUT_BASE}.[pgm|yaml|png]"
echo "[smoketest] slam_config: ${SLAM_CONFIG}"

bash "$(dirname "$0")/cleanup_sim.sh"
bash "$(dirname "$0")/check_microxrce_agent.sh"

set +e
ros2 launch tof_slam_sim px4_nav2_explore.launch.py \
  world:="${WORLD}" \
  monitor:=true \
  slam_config:="${SLAM_CONFIG}" \
  rviz:=false \
  gz_gui:=false &
LAUNCH_PID=$!
set -e

cleanup() {
  echo "[smoketest] Cleaning up..."
  bash "$(dirname "$0")/cleanup_sim.sh" || true
}
trap cleanup EXIT

sleep "${DURATION_SEC}"

echo "[smoketest] Saving map..."
ros2 run nav2_map_server map_saver_cli -f "${OUT_BASE}" --ros-args -p use_sim_time:=true

PGM="${OUT_BASE}.pgm"
PNG="${OUT_BASE}.png"
PNG_X4="${OUT_BASE}_x4.png"

echo "[smoketest] Converting ${PGM} -> ${PNG}"
PGM="${PGM}" PNG="${PNG}" PNG_X4="${PNG_X4}" python3 - <<'PY'
import cv2
import os
import sys

pgm = os.environ["PGM"]
png = os.environ["PNG"]
png_x4 = os.environ["PNG_X4"]

img = cv2.imread(pgm, cv2.IMREAD_UNCHANGED)
if img is None:
    print(f"[smoketest] ERROR: failed to read {pgm}", file=sys.stderr)
    sys.exit(1)

cv2.imwrite(png, img)

h, w = img.shape[:2]
img_x4 = cv2.resize(img, (w * 4, h * 4), interpolation=cv2.INTER_NEAREST)
cv2.imwrite(png_x4, img_x4)
print(f"[smoketest] wrote {png} and {png_x4}")
PY

echo "[smoketest] Done. (Stopping sim...)"
kill -INT "${LAUNCH_PID}" 2>/dev/null || true

# `ros2 launch` can occasionally ignore SIGINT or take a long time to unwind if
# Gazebo/PX4 processes are busy. Don't hang the smoketest forever; escalate.
for _ in $(seq 1 30); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoketest] ros2 launch still running; sending SIGTERM..."
  kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  sleep 5
fi

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoketest] ros2 launch still running; sending SIGKILL..."
  kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
fi

wait "${LAUNCH_PID}" 2>/dev/null || true
