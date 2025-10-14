#!/usr/bin/env bash
set -euo pipefail

# ========= You can override these when running =========
MODEL="${MODEL:-}"          # e.g. MODEL=tof_slam_quadcopter
WORLD="${WORLD:-}"          # e.g. WORLD=playfield
ROS_TWIST_X="${ROS_TWIST_X:-1.0}"   # ROS test linear x
GZ_TWIST_X="${GZ_TWIST_X:-1.0}"     # GZ test linear x
PAUSE_FIX="${PAUSE_FIX:-1}"         # 1 = force unpause world
DELAY="${DELAY:-0.6}"               # seconds to wait between readings
# =======================================================

ok()   { printf "\033[32m%s\033[0m\n" "$*"; }
warn() { printf "\033[33m%s\033[0m\n" "$*"; }
err()  { printf "\033[31m%s\033[0m\n" "$*"; }
info() { printf "\033[36m%s\033[0m\n" "$*"; }
ts()   { date +"%H:%M:%S"; }

need() {
  local miss=0
  for c in ros2 gz grep awk sed timeout; do
    command -v "$c" >/dev/null 2>&1 || { err "[!] Missing tool: $c"; miss=1; }
  done
  if [ $miss -eq 1 ]; then
    err "Run this from the same pixi shell you launch the sim in (robostack:jazzy)."
    exit 1
  fi
}

env_summary() {
  info "[ENV] ROS_DISTRO=${ROS_DISTRO:-<unset>}  RMW=${RMW_IMPLEMENTATION:-<unset>}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  info "[ENV] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-<unset>}"
  local logdir="${RCL_LOGGING_DIR:-${ROS_LOG_DIR:-/tmp/ros_logs}}"
  mkdir -p "$logdir" >/dev/null 2>&1 || true
  [ -w "$logdir" ] && ok "[LOG] Using logging dir: $logdir" || warn "[LOG] $logdir not writable"
}

detect_world_model() {
  if [ -z "${WORLD}" ]; then
    WORLD="$(gz topic -l 2>/dev/null | sed -n 's|^/world/\([^/]*\)/.*$|\1|p' | head -n1 || true)"
    [ -n "$WORLD" ] && info "[i] Detected WORLD=$WORLD" || warn "[!] Could not detect WORLD (set WORLD=...)"
  fi
  if [ -z "${MODEL}" ]; then
    local line name
    line="$(gz topic -l 2>/dev/null | grep -E '/model/.*/cmd_vel' | head -n1 || true)"
    name="$(echo "$line" | sed -n 's|.*/model/\([^/]*\)/cmd_vel.*|\1|p')"
    [ -n "$name" ] && MODEL="$name" && info "[i] Detected MODEL=$MODEL" || warn "[!] Could not detect MODEL (set MODEL=...)"
  fi
  [ -z "$WORLD" ] && { err "[X] WORLD unknown"; exit 1; }
  [ -z "$MODEL" ] && { err "[X] MODEL unknown"; exit 1; }
}

unpause_world() {
  if [ "$PAUSE_FIX" = "1" ]; then
    info "[gz] Forcing world unpause"
    gz service -s "/world/${WORLD}/control" \
      --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
      --timeout 1000 --req 'pause:false' >/dev/null 2>&1 || true
  fi
}

gz_topic_exists() { gz topic -l | grep -qx "$1"; }

gz_endpoints() {
  local t="$1"
  echo "=== GZ topic: $t"
  if gz_topic_exists "$t"; then
    gz topic -i -t "$t" || true
  else
    echo "MISSING"
  fi
  echo
}

gz_pulse() {
  local t="$1" vx="$2"
  info "[gz] Pulse: $t  linear.x=$vx"
  gz topic -t "$t" -m gz.msgs.Twist -p "linear: { x: ${vx} }" >/dev/null 2>&1 || true
  sleep 0.35
  gz topic -t "$t" -m gz.msgs.Twist -p "linear: { x: 0.0 }" >/dev/null 2>&1 || true
}

# ---- read one odom sample (ROS) -> "x y z" or empty
ros_odom_xyz() {
  # Try --once (newer) then -n 1, else fallback to timeout
  local out
  if ros2 topic echo --once /odom 2>/dev/null | head -n 200 >/tmp/_odom_once.$$ ; then
    out="$(awk '
      /position:/   {pos=1; next}
      pos && $1=="x:" {x=$2}
      pos && $1=="y:" {y=$2}
      pos && $1=="z:" {z=$2; print x, y, z; exit}
    ' /tmp/_odom_once.$$)"
    rm -f /tmp/_odom_once.$$
  else
    if ros2 topic echo -n 1 /odom 2>/dev/null | head -n 200 >/tmp/_odom_n.$$ ; then
      out="$(awk '
        /position:/   {pos=1; next}
        pos && $1=="x:" {x=$2}
        pos && $1=="y:" {y=$2}
        pos && $1=="z:" {z=$2; print x, y, z; exit}
      ' /tmp/_odom_n.$$)"
      rm -f /tmp/_odom_n.$$
    else
      # Fallback: brief timeout stream
      timeout 1.5 ros2 topic echo /odom 2>/dev/null | head -n 200 >/tmp/_odom_to.$$ || true
      out="$(awk '
        /position:/   {pos=1; next}
        pos && $1=="x:" {x=$2}
        pos && $1=="y:" {y=$2}
        pos && $1=="z:" {z=$2; print x, y, z; exit}
      ' /tmp/_odom_to.$$)"
      rm -f /tmp/_odom_to.$$
    fi
  fi
  echo "$out"
}

# ---- read one odom sample (GZ) -> "x y z" or empty
gz_odom_xyz() {
  local topic="/model/${MODEL}/odometry"
  timeout 1.5 gz topic -e -t "$topic" 2>/dev/null | head -n 200 >/tmp/_gz_odom.$$
  local out
  out="$(awk '
    /position[[:space:]]*\{/ {pos=1; next}
    pos && $1=="x:" {x=$2}
    pos && $1=="y:" {y=$2}
    pos && $1=="z:" {z=$2; print x, y, z; exit}
  ' /tmp/_gz_odom.$$)"
  rm -f /tmp/_gz_odom.$$
  echo "$out"
}

dist3() { awk -v ax="$1" -v ay="$2" -v az="$3" -v bx="$4" -v by="$5" -v bz="$6" 'BEGIN{dx=bx-ax; dy=by-ay; dz=bz-az; print sqrt(dx*dx+dy*dy+dz*dz)}'; }

check_bridge_and_plugins() {
  info "[check] Bridge + plugins visibility"
  gz_endpoints "/model/${MODEL}/cmd_vel"
  gz_endpoints "/model/${MODEL}/odometry"
  info "[ros] Nodes of interest:"
  ros2 node list 2>/dev/null | grep -E 'parameter_bridge|auto_pilot|python' | sed 's/^/  /' || true
  info "[ros] /cmd_vel"
  ros2 topic info /cmd_vel -v 2>/dev/null | sed 's/^/  /' || warn "no /cmd_vel"
  info "[ros] /odom"
  ros2 topic info /odom -v 2>/dev/null | sed 's/^/  /' || warn "no /odom"
}

ros_motion_test() {
  info "[TEST] ROS path: /cmd_vel -> bridge -> VelocityControl"
  local before after d
  before="$(ros_odom_xyz)"
  [ -z "$before" ] && warn "[ros] Could not sample /odom (try Gazebo test next)"
  info "[ros] publish 1-shot Twist: linear.x=${ROS_TWIST_X}"
  # fire-and-forget one message
  ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: ${ROS_TWIST_X}}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep "$DELAY"
  after="$(ros_odom_xyz)"
  if [ -n "$before" ] && [ -n "$after" ]; then
    d="$(dist3 $before $after)"
    info "[ros] odom delta: $d"
    awk -v dd="$d" 'BEGIN{ if (dd>0.01) exit 0; else exit 1 }' && ok "[✓] Movement via ROS path detected" || warn "[!] No significant movement via ROS path"
  else
    warn "[!] ROS odom reading failed; continuing with Gazebo-native test"
  fi
}

gz_motion_test() {
  info "[TEST] Gazebo-native path: /model/${MODEL}/cmd_vel -> VelocityControl"
  local before after d
  before="$(gz_odom_xyz)"
  [ -z "$before" ] && warn "[gz] Could not sample /model/${MODEL}/odometry; is OdometryPublisher loaded?"
  gz_pulse "/model/${MODEL}/cmd_vel" "$GZ_TWIST_X"
  sleep "$DELAY"
  after="$(gz_odom_xyz)"
  if [ -n "$before" ] && [ -n "$after" ]; then
    d="$(dist3 $before $after)"
    info "[gz] odometry delta: $d"
    awk -v dd="$d" 'BEGIN{ if (dd>0.01) exit 0; else exit 1 }' && ok "[✓] Movement via Gazebo-native path detected" || err "[X] No movement via Gazebo path (plugins loaded but pose not changing)"
  else
    warn "[!] Could not compute delta in Gazebo test"
  fi
}

final_diagnosis() {
  echo
  info "===== SUMMARY ====="
  echo "• If Gazebo test moved but ROS test didn’t → bridge/remap issue on /cmd_vel."
  echo "• If ROS test moved but you don’t see it in GUI → camera not following; right-click model → Follow."
  echo "• If neither test moved but endpoints exist → world paused, wrong model name, or plugin filenames mismatched in the SDF actually loaded."
  echo "• Quick manual nudges you can try:"
  echo "    gz topic -t /model/${MODEL}/cmd_vel -m gz.msgs.Twist -p 'linear: { x: 1.0 }'"
  echo "    gz topic -e -t /model/${MODEL}/odometry | head"
  echo "• Ensure autopilot isn’t constantly publishing Z velocity if gravity=false (set AP_LIN_Z=0.0)."
  echo
}

main() {
  echo "$(ts) ===== gz_move_doctor start ====="
  need
  env_summary
  detect_world_model
  unpause_world
  check_bridge_and_plugins
  ros_motion_test
  gz_motion_test
  final_diagnosis
}
main "$@"
