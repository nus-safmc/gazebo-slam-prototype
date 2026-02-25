#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  bash scripts/verify_tf_rates.sh [--namespace robot] [--hz-window-sec N] [--tf-echo-sec N]

Runs quick TF + topic-rate checks while the sim is running.

Examples:
  pixi run -e jazzy bash scripts/verify_tf_rates.sh
  pixi run -e jazzy bash scripts/verify_tf_rates.sh --namespace robot --hz-window-sec 5
EOF
}

NS="robot"
HZ_WINDOW_SEC="5"
TF_ECHO_SEC="3"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --namespace) NS="${2:-}"; shift 2 ;;
    --hz-window-sec) HZ_WINDOW_SEC="${2:-}"; shift 2 ;;
    --tf-echo-sec) TF_ECHO_SEC="${2:-}"; shift 2 ;;
    *) echo "[verify] Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[verify] ros2 is not on PATH. Run via pixi:" >&2
  echo "  pixi run -e jazzy bash scripts/verify_tf_rates.sh" >&2
  exit 127
fi

MAP_FRAME="${NS}/map"
ODOM_FRAME="${NS}/odom"
BASE_FOOT_FRAME="${NS}/base_footprint"

echo "[verify] time: $(date -Is)"
echo "[verify] frames: ${MAP_FRAME} -> ${ODOM_FRAME} -> ${BASE_FOOT_FRAME}"

echo
echo "[verify] TF (should print transforms, not errors):"
timeout "${TF_ECHO_SEC}s" ros2 run tf2_ros tf2_echo "${ODOM_FRAME}" "${BASE_FOOT_FRAME}" || true
timeout "${TF_ECHO_SEC}s" ros2 run tf2_ros tf2_echo "${MAP_FRAME}" "${ODOM_FRAME}" || true

echo
echo "[verify] Topic rates (expect non-zero Hz):"
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /odom || true
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /tf || true
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /scan_merged || true
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /clock || true
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /cmd_vel || true
timeout "${HZ_WINDOW_SEC}s" ros2 topic hz /map || true

echo
echo "[verify] PX4 (optional; may be absent if not running):"
timeout 2s ros2 topic hz /fmu/out/vehicle_odometry || true
timeout 2s ros2 topic hz /fmu/in/vehicle_visual_odometry || true
timeout 2s ros2 topic echo --once /fmu/out/vehicle_status || true
timeout 2s ros2 topic echo --once /fmu/out/failsafe_flags || true

echo
echo "[verify] Done."
