#!/usr/bin/env bash
set -euo pipefail

# -------- Config (override when running) --------
MODEL="${MODEL:-}"          # e.g. tof_slam_quadcopter
WORLD="${WORLD:-}"          # e.g. playfield
LOOP_SECS="${LOOP_SECS:-5}" # refresh interval
INJECT="${INJECT:-0}"       # 1 = send a Gazebo-native Twist test each loop
TWAIT="${TWAIT:-2}"         # seconds for any timeouts (best effort)
# ------------------------------------------------

have_cmd() { command -v "$1" >/dev/null 2>&1; }
ts() { date +"%H:%M:%S"; }
say() { printf "%s %s\n" "$(ts)" "$*"; }
col() { local c="$1"; shift
  case "$c" in
    ok)   printf "\033[32m%s\033[0m\n" "$*";;
    warn) printf "\033[33m%s\033[0m\n" "$*";;
    err)  printf "\033[31m%s\033[0m\n" "$*";;
    info) printf "\033[36m%s\033[0m\n" "$*";;
    *)    echo "$*";;
  esac
}

need_tools() {
  local miss=0
  for c in ros2 gz grep sed awk; do
    if ! have_cmd "$c"; then col err "[!] Missing tool: $c"; miss=1; fi
  done
  if [ "$miss" -eq 1 ]; then
    col err "Install missing tools and run inside the SAME pixi shell you use to launch."
    exit 1
  fi
}

autodetect_names() {
  if [ -z "${MODEL}" ]; then
    local line name
    line="$(gz topic -l 2>/dev/null | grep -E '/model/.*/cmd_vel' || true)"
    name="$(echo "$line" | sed -n 's|.*/model/\([^/]*\)/cmd_vel.*|\1|p' | head -n1)"
    if [ -n "$name" ]; then MODEL="$name"; col info "[i] Detected MODEL=$MODEL"; else col warn "[!] Could not detect MODEL; set MODEL=..."; fi
  fi
  if [ -z "${WORLD}" ]; then
    WORLD="$(gz topic -l 2>/dev/null | sed -n 's|^/world/\([^/]*\)/.*$|\1|p' | head -n1)"
    if [ -n "$WORLD" ]; then col info "[i] Detected WORLD=$WORLD"; else col warn "[!] Could not detect WORLD; set WORLD=..."; fi
  fi
}

ensure_log_dir() {
  local want="${RCL_LOGGING_DIR:-${ROS_LOG_DIR:-/tmp/ros_logs}}"
  if [ ! -d "$want" ]; then
    mkdir -p "$want" 2>/dev/null || true
  fi
  if [ -w "$want" ]; then
    col ok "[LOG] Using logging dir: $want"
  else
    col warn "[LOG] $want is not writable; export RCL_LOGGING_DIR=/tmp/ros_logs (or similar) before launching"
  fi
}

gz_topic_exists() { gz topic -l | grep -qx "$1"; }

gz_peek() {
  local t="$1"
  echo "=== GZ topic: $t"
  if gz_topic_exists "$t"; then
    gz topic -i -t "$t" 2>/dev/null || true
  else
    echo "MISSING"
  fi
  echo
}

gz_inject_twist() {
  local t="$1"
  col info "[GZ] Injecting small Twist on $t"
  gz topic -t "$t" -m gz.msgs.Twist -p "linear: { x: 0.6 }" >/dev/null 2>&1 || true
  sleep 0.4
  gz topic -t "$t" -m gz.msgs.Twist -p "linear: { x: 0.0 }" >/dev/null 2>&1 || true
}

ros_topic_exists() { ros2 topic list | grep -qx "$1"; }

ros_info() {
  local t="$1"
  if ros_topic_exists "$t"; then
    col ok "[ROS] $t exists"
    ros2 topic info "$t" -v | sed 's/^/      /'
  else
    col err "[ROS] MISSING: $t"
  fi
}

print_nodes() {
  col info "[ROS] Nodes (bridges & autopilot of interest):"
  ros2 node list 2>/dev/null | grep -E 'parameter_bridge|auto_pilot|python' | sed 's/^/   /' || true
}

print_env() {
  col info "[ENV] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  col info "[ENV] RCL_LOGGING_DIR=${RCL_LOGGING_DIR:-<unset>}  ROS_LOG_DIR=${ROS_LOG_DIR:-<unset>}"
  col info "[ENV] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-<unset>}"
}

say "===== gz_ros_diag_v2 start ====="
need_tools
autodetect_names
print_env
ensure_log_dir

if [ -z "$MODEL" ] || [ -z "$WORLD" ]; then
  col warn "[!] Tip: you can run like  MODEL=tof_slam_quadcopter WORLD=playfield ./gz_ros_diag_v2.sh"
fi

while true; do
  echo
  col info "---- $(ts) MODEL=${MODEL:-<?>}  WORLD=${WORLD:-<?>} ----"

  # Gazebo side
  gz_peek "/model/${MODEL}/cmd_vel"
  gz_peek "/model/${MODEL}/odometry"
  if [ "$INJECT" = "1" ] && gz_topic_exists "/model/${MODEL}/cmd_vel"; then
    gz_inject_twist "/model/${MODEL}/cmd_vel"
  fi

  # ROS side
  print_nodes
  ros_info "/cmd_vel"
  ros_info "/odom"

  # Depth quick probe (front only)
  if [ -n "${MODEL:-}" ] && [ -n "${WORLD:-}" ]; then
    base="/world/${WORLD}/model/${MODEL}/link/tof_front/sensor/depth_camera"
    for tail in image depth depth_image; do
      if gz_topic_exists "${base}/${tail}"; then
        col ok "[GZ] Depth present: ${base}/${tail}"
        break
      fi
    done
  fi
  if ros_topic_exists "/tof_front/depth"; then
    col ok "[ROS] Depth mapped: /tof_front/depth"
  else
    col warn "[ROS] Depth mapped: /tof_front/depth (missing)"
  fi

  echo "----------------------------------------------"
  sleep "$LOOP_SECS"
done
