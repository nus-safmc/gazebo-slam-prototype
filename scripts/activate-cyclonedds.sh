#!/bin/bash
# Pixi activation script to configure CycloneDDS for ROS 2
# This prevents FastDDS hanging issues in RoboStack environments
# See: https://github.com/RoboStack/ros-jazzy/issues/57

# Save any existing RMW_IMPLEMENTATION value for restoration
export _PIXI_BACKUP_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}"

# Force CycloneDDS to avoid FastDDS hanging issues
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

echo "ROS 2 DDS configured: RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
