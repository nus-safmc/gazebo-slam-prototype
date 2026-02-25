#!/usr/bin/env bash
set -euo pipefail

TOPIC="${1:-/map}"
UPDATES_TOPIC="${2:-/map_updates}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[verify_map_runtime] ros2 is not on PATH. Run via pixi:" >&2
  echo "  pixi run -e jazzy bash scripts/verify_map_runtime.sh" >&2
  exit 127
fi

echo "[verify_map_runtime] Checking publishers for ${TOPIC} ..."
INFO_OUT="$(ros2 topic info -v "${TOPIC}" 2>/dev/null || true)"
echo "${INFO_OUT}"

PUB_COUNT="$(
  printf '%s\n' "${INFO_OUT}" | awk '/Publisher count:/ {print $3; exit}' | tr -d '\r'
)"
if [[ -n "${PUB_COUNT}" ]]; then
  if [[ "${PUB_COUNT}" -gt 1 ]]; then
    echo "[verify_map_runtime] ERROR: ${TOPIC} has ${PUB_COUNT} publishers (should be 1). Run cleanup:" >&2
    echo "  pixi run -e jazzy cleanup_sim" >&2
    exit 2
  fi
fi

echo
echo "[verify_map_runtime] Checking publishers for ${UPDATES_TOPIC} ..."
ros2 topic info -v "${UPDATES_TOPIC}" 2>/dev/null || true

echo
echo "[verify_map_runtime] Inspecting ${TOPIC} message stats ..."
python3 scripts/inspect_map_once.py --topic "${TOPIC}" --timeout-sec 5.0 --border-cells 5

