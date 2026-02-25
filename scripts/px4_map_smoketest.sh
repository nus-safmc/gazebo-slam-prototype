#!/usr/bin/env bash
# Backward compatibility wrapper - delegates to organized script
exec "$(dirname "$0")/test/px4_map_smoketest.sh" "$@"
