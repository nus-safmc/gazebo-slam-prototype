#!/usr/bin/env bash
# Backward compatibility wrapper - delegates to organized script
exec "$(dirname "$0")/core/tail_latest_log.sh" "$@"