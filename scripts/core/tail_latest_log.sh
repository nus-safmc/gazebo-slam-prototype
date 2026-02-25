#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  bash scripts/tail_latest_log.sh <prefix> [--raw] [tail args...]

Examples:
  bash scripts/tail_latest_log.sh px4_nav2_explore
  bash scripts/tail_latest_log.sh px4_nav2_explore --raw -n 200 -f
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

PREFIX="$1"
shift

TAIL_RAW=0
if [[ "${1:-}" == "--raw" ]]; then
  TAIL_RAW=1
  shift
fi

LOG_ROOT="${LOG_ROOT:-$(pwd)/log/run_logs}"
RAW_LATEST="${LOG_ROOT}/${PREFIX}_latest.log"
FILTERED_LATEST="${LOG_ROOT}/${PREFIX}_latest.filtered.log"

FILE="$RAW_LATEST"
if [[ "$TAIL_RAW" -eq 0 && -f "$FILTERED_LATEST" ]]; then
  FILE="$FILTERED_LATEST"
fi

if [[ ! -e "$FILE" ]]; then
  echo "error: log file not found: $FILE" >&2
  echo "hint: run a logged task first, e.g.:" >&2
  echo "  pixi run -e jazzy ${PREFIX}_log" >&2
  exit 2
fi

exec tail -f "$FILE" "$@"

