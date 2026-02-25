#!/usr/bin/env bash
set -euo pipefail

# Build and install MicroXRCEAgent into the active pixi/conda environment.
#
# Usage:
#   pixi run -e jazzy bash scripts/build_microxrce_agent.sh
#
# Installs to:
#   $CONDA_PREFIX/bin/MicroXRCEAgent

if [[ -z "${CONDA_PREFIX:-}" ]]; then
  echo "[px4] error: CONDA_PREFIX is not set. Run inside pixi, e.g.:" >&2
  echo "  pixi run -e jazzy bash scripts/build_microxrce_agent.sh" >&2
  exit 2
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "[px4] MicroXRCEAgent already present: $(command -v MicroXRCEAgent)"
  exit 0
fi

REPO_URL="https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
SRC_DIR="$(pwd)/build/microxrce_dds_agent_src"
BUILD_DIR="${SRC_DIR}/build"

echo "[px4] Building MicroXRCEAgent from source..."
echo "[px4]   repo:  ${REPO_URL}"
echo "[px4]   src:   ${SRC_DIR}"
echo "[px4]   build: ${BUILD_DIR}"
echo "[px4]   dest:  ${CONDA_PREFIX}"

mkdir -p "$(dirname "$SRC_DIR")"
if [[ ! -d "${SRC_DIR}/.git" ]]; then
  git clone --depth 1 "${REPO_URL}" "${SRC_DIR}"
fi

OS="$(uname -s)"
NJOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

CMAKE_EXTRA_ARGS=()
case "${OS}" in
  Darwin)
    # Conda cross-compiler doesn't expose CMAKE_SYSTEM_NAME, so cmake
    # can't auto-disable the Linux-only CAN transport.  Set both explicitly.
    CMAKE_EXTRA_ARGS+=(
      -DCMAKE_SYSTEM_NAME=Darwin
      -DUAGENT_SOCKETCAN_PROFILE=OFF
    )
    ;;
  Linux)
    CMAKE_EXTRA_ARGS+=( -DCMAKE_SYSTEM_NAME=Linux )
    ;;
esac

echo "[px4]   os:    ${OS}"
echo "[px4]   jobs:  ${NJOBS}"

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CONDA_PREFIX}" \
  "${CMAKE_EXTRA_ARGS[@]}"

cmake --build "${BUILD_DIR}" -j"${NJOBS}"
cmake --install "${BUILD_DIR}"

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "[px4] MicroXRCEAgent installed: $(command -v MicroXRCEAgent)"
  exit 0
fi

echo "[px4] error: install completed, but MicroXRCEAgent is still not on PATH." >&2
echo "[px4] expected: ${CONDA_PREFIX}/bin/MicroXRCEAgent" >&2
exit 2

