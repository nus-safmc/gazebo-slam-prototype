#!/usr/bin/env bash
# Backward compatibility wrapper - delegates to organized script
exec "$(dirname "$0")/px4/check_microxrce_agent.sh" "$@"
