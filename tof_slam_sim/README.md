# ToF SLAM Simulation

This package provides a simulation environment for validating 2D SLAM using an array of 8 simulated VL53L7CX ToF sensors in Gazebo Harmonic.

## Features

- 8× VL53L7CX ToF sensor simulation using depth cameras
- 15 cm × 15 cm quadcopter model
- 20 m × 20 m competition playfield with obstacles
- ROS 2 bridge for sensor data
- Depth to LaserScan conversion
- 360° scan merging
- SLAM validation using slam_toolbox

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- ros_gz packages
- slam_toolbox

## Building

```bash
# From your ROS 2 workspace root
colcon build --symlink-install --packages-select tof_slam_sim
```

## Usage

1. Launch the simulation with ROS bridge:
```bash
ros2 launch tof_slam_sim sim_with_bridge.launch.py
```

2. Launch SLAM for testing:
```bash
ros2 launch tof_slam_sim slam_test.launch.py
```

## Package Structure

- `models/` - Gazebo model definitions
  - `vl53l7cx/` - ToF sensor model
  - `quadcopter/` - Drone with 8 sensors
- `worlds/` - Gazebo world definitions
- `launch/` - ROS 2 launch files
- `src/` - Python nodes
  - `tof8x8_to_scan.py` - Depth to LaserScan conversion
  - `scan_merger.py` - 360° scan merger
  - `test_controller.py` - Simple robot controller
- `config/` - Configuration files

## License

Apache 2.0
