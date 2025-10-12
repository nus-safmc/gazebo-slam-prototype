# Gazebo SLAM Agent Code Structure & Setup Guide

## Project Architecture Overview

This project implements a ROS 2 Gazebo simulation for 2D SLAM using 8 VL53L7CX ToF sensors mounted on a quadcopter drone. The system uses RoboStack + PIXI for environment management and CycloneDDS for ROS communication.

### Core Components
- **Gazebo Harmonic**: Physics simulation with custom drone model
- **ROS 2 Jazzy**: Robotics middleware via RoboStack PIXI environment
- **8× VL53L7CX Sensors**: Simulated depth cameras arranged in circular pattern
- **ros_gz_bridge**: Bidirectional topic bridging between Gazebo and ROS
- **CycloneDDS**: DDS implementation for reliable ROS communication

## Environment Setup & Package Management

### PIXI Environment Management
```bash
# Activate ROS Jazzy environment for all commands
pixi run -e jazzy <command>

# Enter interactive ROS environment shell
pixi shell -e jazzy

# Key environment variables (automatically set)
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# GZ_SIM_RESOURCE_PATH includes project models/worlds
```

### RoboStack ROS Distribution
- **Isolated Environment**: ROS 2 Jazzy in dedicated conda environment
- **Pre-configured CycloneDDS**: No manual DDS setup required
- **Cross-platform**: Same workflow on macOS, Linux, Windows
- **Dependency Management**: Automatic resolution of ROS package dependencies

## Project Code Structure

### Package Organization (`tof_slam_sim/`)
```
tof_slam_sim/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata & dependencies
├── models/                     # Gazebo model definitions
│   ├── vl53l7cx/              # ToF sensor model (depth camera)
│   ├── quadcopter/            # Drone with 8 sensor mount points
│   ├── inner_wall/            # Maze obstacles
│   ├── perimeter_wall/        # Boundary walls
│   └── pillar/                # Vertical obstacles
├── worlds/                    # Gazebo world files
│   └── playfield.sdf         # 20m×20m competition arena
├── launch/                    # ROS 2 launch configurations
│   ├── sim_with_bridge.launch.py    # Full simulation + ROS bridge
│   └── slam_test.launch.py          # SLAM validation setup
├── src/                       # Python ROS nodes
│   ├── tof8x8_to_scan.py      # Depth image → LaserScan conversion
│   ├── scan_merger.py        # 360° scan merging from 8 sensors
│   └── test_controller.py     # Robot control logic
└── config/                    # Configuration files
    ├── slam_toolbox.yaml      # SLAM mapping parameters
    └── slam.rviz             # RViz visualization config
```

### Key ROS Nodes Architecture

#### Sensor Processing Pipeline
```python
# tof8x8_to_scan.py - Individual sensor processing
# Converts 8×8 depth images to LaserScan messages
class ToF8x8ToScan(Node):
    def __init__(self, sensor_name):
        super().__init__(f'tof_{sensor_name}_to_scan')
        # Subscribe to depth image from Gazebo bridge
        # Publish LaserScan with appropriate angle offset
```

#### Scan Merging Node
```python
# scan_merger.py - Multi-sensor fusion
# Combines 8 individual scans into 360° point cloud
class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        # Subscribe to all 8 LaserScan topics
        # Publish merged scan for SLAM
```

#### Robot Controller
```python
# test_controller.py - Motion control
# Provides cmd_vel interface for robot navigation
class TestController(Node):
    def __init__(self):
        super().__init__('test_controller')
        # Publish velocity commands for testing
```

## Building & Compilation

### PIXI Build System
```bash
# Initial setup and build
pixi run -e jazzy build

# The build system:
# 1. Downloads ROS 2 Jazzy + dependencies
# 2. Compiles Python nodes (automatic)
# 3. Installs to .pixi/envs/jazzy/
# 4. No separate build/install directories

# Rebuild after code changes
pixi run -e jazzy build

# Clean build artifacts
pixi run -e jazzy clean

# Full rebuild (clean + build)
pixi run -e jazzy rebuild
```

### Build Output Structure
```
.pixi/envs/jazzy/
├── lib/python3.12/site-packages/tof_slam_sim/  # Python modules (symlinked)
│   ├── __init__.py
│   ├── scan_merger.py
│   ├── test_controller.py
│   └── tof8x8_to_scan.py
├── share/tof_slam_sim/                        # Launch files, configs, models
│   ├── launch/                               # Launch configurations
│   ├── config/                               # Configuration files
│   ├── models/                               # Gazebo model files
│   ├── worlds/                               # Gazebo world files
│   └── package.xml                           # Package metadata (symlinked)
├── share/ament_index/...                      # ROS package registry
└── bin/                                       # Generated executables (if any)
```

## Launch File Architecture

### Main Simulation Launch (`sim_with_bridge.launch.py`)
```python
def generate_launch_description():
    # 1. Environment Setup
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_models, ':', pkg_worlds]
    )

    # 2. Platform-Specific Gazebo Launch
    is_macos = platform.system() == 'Darwin'
    if is_macos:
        gz_sim = [gz_sim_server, gz_sim_gui]  # Separate processes
    else:
        gz_sim = gz_sim_combined               # Single process

    # 3. ROS-Gazebo Bridges (16 total: 8 sensors × 2 topics each)
    bridge_configs = []
    sensor_names = ['front', 'front_right', 'right', 'back_right',
                   'back', 'back_left', 'left', 'front_left']

    for i, name in enumerate(sensor_names):
        # Depth image bridge
        bridge_configs.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_image_{name}',
            parameters=[{
                'gz_topic': f'/model/robot/model/tof_{name}/link/sensor/camera/image',
                'ros_topic': f'/tof_{name}/image',
                'gz_type': 'gz.msgs.Image',
                'ros_type': 'sensor_msgs/msg/Image',
                'lazy': True
            }]
        ))
        # Camera info bridge
        bridge_configs.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_info_{name}',
            parameters=[{
                'gz_topic': f'/model/robot/model/tof_{name}/link/sensor/camera/camera_info',
                'ros_topic': f'/tof_{name}/camera_info',
                'gz_type': 'gz.msgs.CameraInfo',
                'ros_type': 'sensor_msgs/msg/CameraInfo',
                'lazy': True
            }]
        ))

    # 4. Robot Control Bridges
    cmd_vel_bridge = Node(package='ros_gz_bridge', ...)
    odom_bridge = Node(package='ros_gz_bridge', ...)

    # 5. Staggered Bridge Startup (TimerAction prevents DDS conflicts)
    ld = LaunchDescription()
    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_sim)

    for i, config in enumerate(bridge_configs):
        ld.add_action(TimerAction(period=i * 0.2, actions=[config]))

    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_bridge)

    return ld
```

## Running the System

### Quick Start Commands
```bash
# 1. Build the project
pixi run -e jazzy build

# 2. Launch full simulation with ROS bridge
pixi run -e jazzy sim

# 3. Launch SLAM testing (in separate terminal)
pixi run -e jazzy slam

# 4. Interactive ROS environment
pixi shell -e jazzy
# Now run ROS commands directly:
ros2 topic list
ros2 launch tof_slam_sim sim_with_bridge.launch.py
```

### System Execution Flow
```
1. Launch File Execution
   ├── Environment Setup (GZ_SIM_RESOURCE_PATH)
   ├── Gazebo World Loading (playfield.sdf)
   ├── Sensor Model Loading (8× VL53L7CX)
   └── ROS-Gazebo Bridge Startup (16 bridges with TimerAction)

2. Topic Architecture
   ├── Gazebo Topics (8 depth + 8 camera_info)
   ├── ROS Topics (bridged from Gazebo)
   ├── Sensor Processing (depth → LaserScan)
   └── SLAM Integration (merged scan → map)
```

### Platform-Specific Execution

#### macOS
```bash
# Gazebo requires separate server and GUI processes
pixi run -e jazzy ros2 launch tof_slam_sim sim_with_bridge.launch.py
# Launches: gz sim -s -r world.sdf + gz sim -g
```

#### Linux
```bash
# Gazebo can combine server and GUI
pixi run -e jazzy ros2 launch tof_slam_sim sim_with_bridge.launch.py
# Launches: gz sim -r world.sdf
```

## Topic Architecture & Data Flow

### Gazebo → ROS Topic Mapping
```
Gazebo Topics (Published by Simulation)          ROS Topics (Bridged)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
/model/robot/odometry                          → /odom
/model/robot/cmd_vel                           → /cmd_vel
/model/robot/model/tof_front/link/sensor/camera/image        → /tof_front/image
/model/robot/model/tof_front/link/sensor/camera/camera_info  → /tof_front/camera_info
/model/robot/model/tof_front_right/link/sensor/camera/image  → /tof_front_right/image
... (similar for all 8 sensors) ...
```

### Sensor Processing Pipeline
```python
# Raw sensor data flow:
/tof_front/image (sensor_msgs/Image) → tof8x8_to_scan.py → /tof_front/scan (sensor_msgs/LaserScan)
/tof_right/image → tof8x8_to_scan.py → /tof_right/scan
...
# All 8 scans → scan_merger.py → /scan_merged (sensor_msgs/LaserScan)
# Merged scan → slam_toolbox → /map (nav_msgs/OccupancyGrid)
```

### Bridge Configuration Details
```python
# Each sensor requires 2 bridges (image + camera_info)
bridge_config = {
    'package': 'ros_gz_bridge',
    'executable': 'parameter_bridge',
    'parameters': [{
        'gz_topic': gazebo_topic_name,
        'ros_topic': ros_topic_name,
        'gz_type': 'gz.msgs.Image',      # or gz.msgs.CameraInfo
        'ros_type': 'sensor_msgs/msg/Image',  # or sensor_msgs/msg/CameraInfo
        'lazy': True  # Only activate when Gazebo topic exists
    }]
}
```

## Configuration Management

### PIXI Environment (`pixi.toml`)
```toml
[environments]
jazzy = { features = ["jazzy", "build"] }

[feature.jazzy.dependencies]
ros-jazzy-desktop = "*"  # Full ROS 2 Jazzy installation

[feature.build.target.unix.tasks]
build = "colcon build --symlink-install --install-base .pixi/envs/jazzy --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython_FIND_VIRTUALENV=ONLY -DPython3_FIND_VIRTUALENV=ONLY"
```

### Package Metadata (`package.xml`)
```xml
<package format="3">
  <name>tof_slam_sim</name>
  <version>0.0.1</version>
  <description>Gazebo SLAM simulation with 8 VL53L7CX sensors</description>

  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>ros_gz_bridge</depend>
  <depend>slam_toolbox</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_world_path="${prefix}/worlds"/>
  </export>
</package>
```

## Development Workflow

### Code Development Cycle
```bash
# 1. Modify source code
vim tof_slam_sim/src/tof8x8_to_scan.py

# 2. Rebuild package
pixi run -e jazzy build

# 3. Test changes
pixi run -e jazzy sim

# 4. Check ROS topics
pixi run -e jazzy ros2 topic list | grep tof

# 5. Debug with logging
pixi run -e jazzy ros2 node list
pixi run -e jazzy ros2 node info /tof_front_to_scan
```

### Adding New Sensors
```python
# 1. Add sensor model to quadcopter.sdf
<include>
  <uri>model://vl53l7cx</uri>
  <name>tof_new_sensor</name>
  <pose relative_to="base_link">x y z roll pitch yaw</pose>
</include>

# 2. Add joint connection
<joint name="tof_new_sensor_joint" type="fixed">
  <parent>base_link</parent>
  <child>tof_new_sensor::link</child>
</joint>

# 3. Add bridges to launch file
sensor_names.append('new_sensor')

# 4. Rebuild and test
pixi run -e jazzy build
pixi run -e jazzy sim
```

### Key Architectural Patterns

#### Environment Isolation
- **PIXI**: Manages isolated ROS environments
- **RoboStack**: Provides conda-packaged ROS distributions
- **CycloneDDS**: Pre-configured RMW implementation

#### Platform Abstraction
- **Launch Files**: Automatic macOS/Linux detection
- **Build System**: Cross-platform compilation via colcon
- **Path Management**: Environment variable abstraction

#### Modular Bridge Architecture
- **Lazy Bridges**: Activate only when topics exist
- **TimerAction**: Prevents DDS domain conflicts
- **Topic Naming**: Consistent Gazebo ↔ ROS mapping

#### Sensor Processing Pipeline
- **Individual Processing**: Each sensor converts depth → LaserScan
- **Angle Offsets**: Proper orientation for 360° coverage
- **Scan Merging**: Combines all sensors into unified scan
- **SLAM Integration**: Standard ROS navigation stack compatibility

This architecture enables scalable multi-sensor SLAM simulation with clean separation between simulation (Gazebo), middleware (ROS 2), and processing (custom nodes).

