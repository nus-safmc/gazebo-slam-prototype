# PX4 Integration

## Overview

The swarm system runs on PX4 SITL (Software-In-The-Loop) drones simulated
in Gazebo Harmonic.  Each drone runs a real PX4 firmware instance with
full EKF2 state estimation, offboard velocity control, and sensor fusion.

## Architecture

```
swarm_control (brain)
├── FrontierServer   -- detects unexplored regions from /map
├── GoalAllocator    -- assigns frontiers to available drones
├── DroneExecutor    -- per-drone FSM, sends Nav2 NavigateToPose goals
├── MissionSupervisor -- global lifecycle management
├── TrafficManager   -- proximity monitoring
└── SwarmDashboard   -- web UI on :8080

Navigation layer (swappable via nav_mode argument)
├── Nav2 (default)   -- path planning + obstacle avoidance → cmd_vel
└── AutoPilot        -- reactive exploration → cmd_vel

PX4 infrastructure (per drone)
├── twist_to_px4_offboard  -- cmd_vel → PX4 OffboardControlMode + TrajectorySetpoint
├── px4_vehicle_odometry_to_odom  -- PX4 VehicleOdometry (NED) → ROS Odometry (ENU)
├── pose_to_px4_visual_odometry   -- Gazebo ground-truth → PX4 EKF2 external vision
├── gz_pose_info_to_pose_stamped  -- Gazebo native pose → ROS PoseStamped
├── tof8x8_to_scan (×8)          -- depth images → LaserScan
├── scan_merger                   -- 8 scans → 360° merged scan
└── PX4 SITL instance             -- real flight controller firmware

Shared
├── swarm_tf_broadcaster  -- publishes TF for all robots
├── swarm_map_fuser       -- fuses all scans into single /map
├── Gazebo server         -- physics simulation
├── ros_gz_bridge         -- depth image + clock bridging
└── MicroXRCE-DDS Agent  -- PX4 ↔ ROS 2 communication
```

## PX4 Control Chain

The key insight is that `twist_to_px4_offboard` sits between the
navigation layer and PX4, making the flight controller transparent
to the swarm_control brain:

1. **FrontierServer** detects frontiers from `/map`
2. **GoalAllocator** assigns a frontier to a DroneExecutor
3. **DroneExecutor** sends a `NavigateToPose` goal to Nav2
4. **Nav2** plans a path and publishes `cmd_vel` (Twist)
5. **twist_to_px4_offboard** converts body-FLU velocity to NED,
   maintains altitude hold, and publishes PX4 `TrajectorySetpoint`
6. **PX4 SITL** executes the velocity command via its flight controller
7. **px4_vehicle_odometry_to_odom** publishes the resulting odometry
   back to ROS for TF and SLAM

## Coordinate Frame Conventions

| System | Position | Orientation |
|--------|----------|-------------|
| PX4    | NED (North-East-Down) | FRD (Forward-Right-Down) |
| ROS    | ENU (East-North-Up)   | FLU (Forward-Left-Up)    |
| Gazebo | ENU                   | FLU                      |

The bridge nodes handle all conversions:
- `px4_vehicle_odometry_to_odom`: NED → ENU position, FRD → FLU quaternion
- `twist_to_px4_offboard`: FLU body velocity → NED world velocity
- `pose_to_px4_visual_odometry`: ENU pose → NED visual odometry

## PX4 EKF2 Configuration

PX4's EKF2 is configured via `px4-rc.params` for indoor SLAM:
- External vision aiding (horizontal position + velocity + yaw)
- Barometer as primary height reference
- No GPS or magnetometer required
- Optical flow and range finder aiding enabled

## Prerequisites

1. **PX4-Autopilot**: Clone and build the PX4 firmware with the
   `gz_x500_small_tof` target (includes TOF-Ring sensor mount).

2. **MicroXRCE-DDS Agent**: Build using `scripts/build_microxrce_agent.sh`.
   This provides the PX4 ↔ ROS 2 transport layer.

3. **px4_msgs**: The ROS 2 message package for PX4 topics.

## Quick Start

```bash
# Build both packages
pixi run -e jazzy build

# Launch full PX4 swarm + swarm_control
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py \
    num_robots:=4 nav_mode:=nav2

# Or with autopilot navigation
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py \
    num_robots:=4 nav_mode:=autopilot
```

## Troubleshooting

### Drones don't arm

Check that `px4-rc.params` is on the PATH and that EKF2 is receiving
external vision data.  Use `ros2 topic hz /fmu/in/vehicle_visual_odometry`
to verify.

### No odometry

Verify MicroXRCE-DDS Agent is running: `scripts/check_microxrce_agent.sh`.
Check `ros2 topic hz /fmu/out/vehicle_odometry`.

### Map not updating

Ensure the `ros_gz_bridge` is publishing depth images.  Check
`ros2 topic hz /depth/tof_1`.  Verify `swarm_map_fuser` is running
and subscribed to all scan topics.
