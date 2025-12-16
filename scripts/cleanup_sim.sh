#!/usr/bin/env bash
set -euo pipefail

echo "[cleanup] Stopping stray Gazebo / bridge processes..."

# NOTE: Use regex tricks ([g]z instead of gz) so pkill doesn't match itself.

# Any Gazebo sim instance (common cause of multiple /clock publishers).
# Use an anchored regex so we don't accidentally match a parent shell command
# that merely contains the text "gz sim".
pkill -f '(^|/)gz sim' 2>/dev/null || true

# Any ros_gz_bridge parameter_bridge (common cause of multiple /clock publishers).
pkill -f '(^|/)ros_gz_bridge/parameter_bridge' 2>/dev/null || true
pkill -f '(^|/)parameter_bridge.*ros_gz_bridge' 2>/dev/null || true

# ros_gz_bridge started by tof_slam_sim/launch/px4_sitl.launch.py
pkill -f '/depth/tof_1@sensor_msgs/msg/Image@gz\.msgs\.Image' 2>/dev/null || true
pkill -f '/model/x500_small_tof_0/pose@geometry_msgs/msg/PoseStamped@gz\.msgs\.Pose' 2>/dev/null || true

# PX4 uXRCE agent (safe to stop when resetting sim state)
pkill -f '[M]icroXRCEAgent udp4 -p 8888' 2>/dev/null || true

# Kill ROS nodes that frequently outlive Gazebo (and then see /clock jump backwards).
pkill -f '(^|/)slam_toolbox/sync_slam_toolbox_node' 2>/dev/null || true
pkill -f '(^|/)rviz2/rviz2.*tof_slam_sim/config/slam\\.rviz' 2>/dev/null || true
pkill -f '(^|/)tof_slam_sim/(auto_pilot|scan_merger|topic_monitor|map_tf_fallback|nav2_frontier_explorer)(\\s|$)' 2>/dev/null || true
pkill -f '(^|/)tof_slam_sim/(tof_to_scan\\.py|pose_to_tf\\.py|tof8x8_to_scan\\.py|tof8x8_to_scan|test_controller\\.py|test_controller)(\\s|$)' 2>/dev/null || true
pkill -f '(^|/)tof_slam_sim/scripts/log_monitor\\.py' 2>/dev/null || true
pkill -f 'ros2 bag record -o bag_' 2>/dev/null || true
pkill -f '(^|/)tf2_ros/static_transform_publisher.*robot/base_footprint' 2>/dev/null || true
pkill -f '(^|/)tf2_ros/static_transform_publisher.*--child-frame-id base_link' 2>/dev/null || true

# Nav2 nodes launched by this repo (match our params filename).
# NOTE: Nav2 bringup may generate rewritten params into /tmp, so don't rely on the params
# filename. We instead match known Nav2 binaries / containers.
pkill -f '(^|/)rclcpp_components/component_container(_isolated)?(\\s|$)' 2>/dev/null || true
# Any Nav2 node binaries (paths like ".../lib/nav2_controller/controller_server").
pkill -f '(^|/)nav2_[^ ]+' 2>/dev/null || true
# Nav2 optionally starts docking via opennav_docking.
pkill -f '(^|/)opennav_docking/opennav_docking(\\s|$)' 2>/dev/null || true
# Also stop bringup launch processes if they are still around.
pkill -f 'nav2_.*\\.launch\\.py' 2>/dev/null || true

# Multi-robot helpers / mapping utilities.
pkill -f '(^|/)tof_slam_sim/(odom_tf_publisher|swarm_map_fuser|tf_namespace_relay|cmd_vel_relay)(\\s|$)' 2>/dev/null || true

# The ROS 2 daemon can cache stale graph info (publisher counts, etc.)
ros2 daemon stop >/dev/null 2>&1 || true

echo "[cleanup] Done."
