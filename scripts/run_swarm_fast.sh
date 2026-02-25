#!/usr/bin/env bash
# Backward compatibility wrapper - delegates to organized script
exec "$(dirname "$0")/swarm/run_swarm_fast.sh" "$@"
