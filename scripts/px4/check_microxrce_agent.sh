#!/usr/bin/env bash
set -euo pipefail

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "[px4] MicroXRCEAgent OK: $(command -v MicroXRCEAgent)"
  exit 0
fi

cat >&2 <<'EOF'
[px4] error: MicroXRCEAgent not found in PATH.

PX4 uXRCE-DDS requires the eProsima Micro-XRCE-DDS Agent (binary: MicroXRCEAgent).

Install options:
  - Ubuntu/Debian: sudo apt update && sudo apt install micro-xrce-dds-agent
  - No sudo (build locally into pixi env): pixi run -e jazzy build_microxrce_agent
  - From source: https://github.com/eProsima/Micro-XRCE-DDS-Agent

After installing, verify with:
  MicroXRCEAgent --help
EOF
exit 2
