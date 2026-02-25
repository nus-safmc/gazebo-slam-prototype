#!/usr/bin/env bash
set -euo pipefail

echo "[cleanup] Stopping stray Gazebo / bridge processes..."

# NOTE: Use regex tricks ([g]z instead of gz) so pkill doesn't match itself.
# `pkill` regex support can vary; keep patterns simple (character classes / substrings).

# Stop any lingering top-level launch processes from this repo first.
# If these stay alive, they may respawn nodes after we pkill them below.
pkill -f '^[r]os2 launch tof_slam_sim' 2>/dev/null || true
pkill -f '^[r]os2 launch swarm_control' 2>/dev/null || true

# Any Gazebo sim instance (common cause of multiple /clock publishers).
# Gazebo can be stubborn to terminate; send SIGINT, then SIGTERM, then SIGKILL.
pkill -INT -f '[g]z sim' 2>/dev/null || true
sleep 0.2
pkill -TERM -f '[g]z sim' 2>/dev/null || true
sleep 0.2
pkill -KILL -f '[g]z sim' 2>/dev/null || true

# Any ros_gz_bridge parameter_bridge (common cause of multiple /clock publishers).
pkill -f '[r]os_gz_bridge/parameter_bridge' 2>/dev/null || true
pkill -f '[p]arameter_bridge.*ros_gz_bridge' 2>/dev/null || true

# ros_gz_bridge started by tof_slam_sim/launch/px4_sitl.launch.py
pkill -f '/depth/tof_1@sensor_msgs/msg/Image@gz\.msgs\.Image' 2>/dev/null || true
pkill -f '/model/x500_small_tof_0/pose@geometry_msgs/msg/PoseStamped@gz\.msgs\.Pose' 2>/dev/null || true

# PX4 SITL binary instances (can leave lock files that block the next run).
pkill -f '[b]in/px4 -i' 2>/dev/null || true
pkill -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
sleep 0.3
# Force-kill any stubborn PX4 instances.
pkill -KILL -f '[b]in/px4 -i' 2>/dev/null || true
pkill -KILL -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
# Remove PX4 lock files so the next run doesn't see "already running".
rm -f /tmp/px4_lock_* 2>/dev/null || true
rm -f /tmp/px4-*lock* 2>/dev/null || true

# PX4 uXRCE agent (safe to stop when resetting sim state)
pkill -f '[M]icroXRCEAgent udp4 -p 8888' 2>/dev/null || true

# Kill anything holding port 8080 (swarm_dashboard web server).
lsof -ti :8080 2>/dev/null | xargs kill -9 2>/dev/null || true

# Kill ROS nodes that frequently outlive Gazebo (and then see /clock jump backwards).
pkill -f '[s]lam_toolbox/sync_slam_toolbox_node' 2>/dev/null || true
pkill -f '[r]viz2/rviz2' 2>/dev/null || true
pkill -f '[t]of_slam_sim/auto_pilot' 2>/dev/null || true
pkill -f '[t]of_slam_sim/scan_merger' 2>/dev/null || true
pkill -f '[t]of_slam_sim/topic_monitor' 2>/dev/null || true
pkill -f '[t]of_slam_sim/map_tf_fallback' 2>/dev/null || true
pkill -f '[t]of_slam_sim/nav2_frontier_explorer' 2>/dev/null || true
pkill -f '[t]of_slam_sim/tof_to_scan\\.py' 2>/dev/null || true
pkill -f '[t]of_slam_sim/pose_to_tf\\.py' 2>/dev/null || true
pkill -f '[t]of_slam_sim/tof8x8_to_scan(\\.py)?' 2>/dev/null || true
pkill -f '[t]of_slam_sim/test_controller(\\.py)?' 2>/dev/null || true
pkill -f '[t]of_slam_sim/scripts/log_monitor\\.py' 2>/dev/null || true
pkill -f 'ros2 bag record -o bag_' 2>/dev/null || true
# Kill all static TF publishers; otherwise they can accumulate across runs and
# cause CycloneDDS "free participant index" failures + confusing TF trees.
pkill -f '[t]f2_ros/static_transform_publisher' 2>/dev/null || true

# Nav2 nodes launched by this repo (match our params filename).
# NOTE: Nav2 bringup may generate rewritten params into /tmp, so don't rely on the params
# filename. We instead match known Nav2 binaries / containers.
pkill -f '[r]clcpp_components/component_container' 2>/dev/null || true
# Any Nav2 node binaries (paths like ".../lib/nav2_controller/controller_server").
pkill -f '[n]av2_' 2>/dev/null || true
# Nav2 optionally starts docking via opennav_docking.
pkill -f '[o]pennav_docking/opennav_docking' 2>/dev/null || true
# Also stop bringup launch processes if they are still around.
pkill -f 'nav2_.*\\.launch\\.py' 2>/dev/null || true

# swarm_control nodes (centralized swarm FSM + web dashboard).
pkill -f '[s]warm_control/frontier_server' 2>/dev/null || true
pkill -f '[s]warm_control/goal_allocator' 2>/dev/null || true
pkill -f '[s]warm_control/mission_supervisor' 2>/dev/null || true
pkill -f '[s]warm_control/traffic_manager' 2>/dev/null || true
pkill -f '[s]warm_control/swarm_dashboard' 2>/dev/null || true
pkill -f '[s]warm_control/drone_executor' 2>/dev/null || true
pkill -f '[s]warm_control\\.frontier_server' 2>/dev/null || true
pkill -f '[s]warm_control\\.goal_allocator' 2>/dev/null || true
pkill -f '[s]warm_control\\.drone_executor' 2>/dev/null || true
pkill -f '[s]warm_control\\.swarm_dashboard' 2>/dev/null || true

# Multi-robot helpers / mapping utilities.
pkill -f '[t]of_slam_sim/odom_tf_publisher' 2>/dev/null || true
pkill -f '[t]of_slam_sim/swarm_tf_broadcaster' 2>/dev/null || true
pkill -f '[t]of_slam_sim\\.swarm_tf_broadcaster' 2>/dev/null || true
pkill -f '[t]of_slam_sim/swarm_map_fuser' 2>/dev/null || true
pkill -f '[t]of_slam_sim/tf_namespace_relay' 2>/dev/null || true
pkill -f '[t]of_slam_sim/cmd_vel_relay' 2>/dev/null || true
pkill -f '[t]of_slam_sim/drone_health_dashboard' 2>/dev/null || true
pkill -f '[t]of_slam_sim/twist_to_px4_offboard' 2>/dev/null || true
pkill -f '[t]of_slam_sim/px4_vehicle_odometry_to_odom' 2>/dev/null || true
pkill -f '[t]of_slam_sim/gz_pose_info_to_pose_stamped' 2>/dev/null || true
pkill -f '[t]of_slam_sim/pose_to_px4_visual_odometry' 2>/dev/null || true

# The ROS 2 daemon can cache stale graph info (publisher counts, etc.)
ros2 daemon stop >/dev/null 2>&1 || true

echo "[cleanup] Done."
