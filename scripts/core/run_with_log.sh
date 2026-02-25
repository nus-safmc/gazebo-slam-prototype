#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  bash scripts/run_with_log.sh <prefix> <command...>

Creates:
  log/run_logs/<prefix>_<timestamp>.log
  log/run_logs/<prefix>_latest.log (symlink)

Optional:
  FILTER_RMW_TYPEHASH=1   # filters rmw_cyclonedds type-hash spam in console + .filtered.log
  LOG_ROOT=...            # override log root directory
EOF
}

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

PREFIX="$1"
shift

LOG_ROOT="${LOG_ROOT:-$(pwd)/log/run_logs}"
mkdir -p "$LOG_ROOT"

TS="$(date -u +'%Y%m%d_%H%M%S')"
BASE="${LOG_ROOT}/${PREFIX}_${TS}"
RAW_LOG="${BASE}.log"
FILTERED_LOG="${BASE}.filtered.log"

RAW_LATEST="${LOG_ROOT}/${PREFIX}_latest.log"
FILTERED_LATEST="${LOG_ROOT}/${PREFIX}_latest.filtered.log"

ln -sfn "$(basename "$RAW_LOG")" "$RAW_LATEST"

echo "[log] command: $*"
echo "[log] raw:      ${RAW_LOG}"

if [[ "${FILTER_RMW_TYPEHASH:-0}" == "1" ]]; then
  ln -sfn "$(basename "$FILTERED_LOG")" "$FILTERED_LATEST"
  echo "[log] filtered: ${FILTERED_LOG}"

  # Keep a raw log (unfiltered), but filter the very noisy CycloneDDS warning spam in the console
  # and in the optional filtered log.
  stdbuf -oL -eL "$@" 2>&1 \
    | tee "$RAW_LOG" \
    | grep --line-buffered -v 'Failed to parse type hash for topic' \
    | tee "$FILTERED_LOG"
else
  stdbuf -oL -eL "$@" 2>&1 | tee "$RAW_LOG"
fi

