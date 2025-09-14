#!/bin/bash

# Force CycloneDDS for ROS 2 to avoid fastdds issues
# See: https://github.com/RoboStack/ros-jazzy/issues/57
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

echo "ROS 2 environment configured with CycloneDDS (RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)"
