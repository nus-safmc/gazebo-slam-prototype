# Swarm Control Package

Centralized control system for multi-robot exploration and mapping.

## Overview

This package implements a centralized swarm control architecture for coordinated frontier exploration with Nav2 navigation:

- **FrontierServer**: Centralized frontier detection from merged map
- **GoalAllocator**: Optimal assignment of frontiers to available drones
- **DroneExecutor**: Per-drone FSM with Nav2 `NavigateToPose` integration
- **MissionSupervisor**: Global mission orchestration with takeoff windows
- **TrafficManager**: Proximity monitoring and collision avoidance
- **SwarmDashboard**: Web-based live dashboard on `http://localhost:8080`

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  FrontierServer │    │  GoalAllocator   │    │ MissionSupervisor│
│                 │    │                  │    │                  │
│ • Detect frontiers│───▶ • Assign drones  │◀───• Global FSM      │
│ • Publish targets│    │ • Greedy scoring │    │ • Takeoff windows│
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  TrafficManager │    │  DroneExecutor   │    │  SwarmDashboard  │
│                 │    │  (×5, per drone) │    │  (web UI :8080)  │
│ • Proximity warn│    │ • FSM: BOOT→AVAIL│    │ • SSE live feed  │
│ • Collision log │    │ • Nav2 goals     │    │ • Drone states   │
└─────────────────┘    └──────────────────┘    └──────────────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │   Nav2 Stack     │
                       │  (per drone)     │
                       └──────────────────┘
```

## Quick Start

### 1. Build the Package
```bash
pixi run -e jazzy build
```

### 2. Launch Full Swarm Test
```bash
# Starts Gazebo (5 robots) + Nav2 stacks + swarm control + web dashboard
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py
```

### 3. Open the Dashboard
Navigate to **http://localhost:8080** in your browser to see:
- Mission state and uptime
- Per-drone FSM states and positions
- Frontier counts and details
- Publish rates and assignment statistics
- Live event log

### 4. Monitor Topics
```bash
# Watch frontier detection
pixi run -e jazzy ros2 topic echo /swarm/frontiers

# Watch assignments
pixi run -e jazzy ros2 topic echo /swarm/assignments

# Watch mission state
pixi run -e jazzy ros2 topic echo /swarm/mission_state
```

## Launch Files

### test_swarm.launch.py (recommended)
Full simulation + swarm control integration test:
- `num_robots` (int, default: 5): Number of drones
- `dashboard` (bool, default: true): Launch web dashboard

### swarm_control.launch.py
Swarm control nodes only (requires simulation infrastructure separately):
- `num_drones` (int, default: 5): Number of drones to control
- `dashboard` (bool, default: true): Launch web dashboard

## Node Parameters

### FrontierServer
- `map_topic`: Map topic to subscribe to (default: `/map`)
- `frontiers_topic`: Topic to publish frontiers (default: `/swarm/frontiers`)
- `min_frontier_cluster_size`: Minimum cells for valid frontier (default: 6)
- `frontier_ttl_sec`: How long to keep frontiers (default: 10.0)

### GoalAllocator
- `assignments_topic`: Topic for assignments (default: `/swarm/assignments`)
- `gain_weight`: Frontier size weight (default: 1.0)
- `cost_weight`: Distance penalty weight (default: 0.35)

### DroneExecutor (per drone)
- `robot_namespace`: Robot namespace (default: `''`)
- `goal_timeout_sec`: Navigation timeout (default: 45.0)

### MissionSupervisor
- `expected_drones`: List of drone names (default: `['robot', 'robot2', 'robot3', 'robot4', 'robot5']`)
- `takeoff_window_duration_sec`: Window duration (default: 30.0)

## ROS Topics

### Publications
- `/swarm/frontiers`: Frontier targets (`PoseStamped`)
- `/swarm/assignments`: Drone assignments (`String`, format: `robot_id:frontier_id:cx:cy`)
- `/swarm/mission_state`: Global mission state (`String`)
- `/swarm/drone_states`: Drone status reports (`String`, format: `robot_id:state:x,y,yaw:assignment`)

### Subscriptions
- `/map`: Occupancy grid for frontier detection
- `/swarm/drone_states`: Drone status for allocation
- `/swarm/assignments`: Assignments for execution

## FSM States

### DroneExecutor States
- `BOOT`: Waiting for Nav2 `NavigateToPose` action server
- `PREFLIGHT`: System checks
- `ARMED`: Ready for takeoff
- `TAKING_OFF`: Ascending (skipped in simulation)
- `STAGING`: Waiting for mission start
- `AVAILABLE`: Ready for assignments
- `EXECUTING_GOAL`: Navigating to frontier via Nav2
- `RECOVERY`: Handling navigation failures
- `RETURNING`: Going home
- `LANDING`: Descending
- `LANDED`: On ground
- `EMERGENCY`: Safety mode (60s Nav2 timeout or critical failure)

### MissionSupervisor States
- `IDLE`: Waiting for drones (auto-starts after 15s)
- `READY`: All drones armed
- `TAKEOFF_WINDOW_A/B`: Staggered takeoff groups
- `RUNNING`: Mission active
- `RETURN_AND_LAND`: Mission ending
- `COMPLETE`: Success
- `ABORT`: Failure (emergency detected)

## Troubleshooting

### Common Issues
1. **No frontiers detected**: Check `/map` topic is publishing
2. **No assignments**: Ensure drones report `AVAILABLE` state
3. **Navigation failures**: Verify TF transforms are working
4. **Dashboard not loading**: Check port 8080 is free; look for node logs

### Debug Commands
```bash
# Check node status
ros2 node list | grep swarm

# Monitor topic traffic
ros2 topic hz /swarm/frontiers
ros2 topic hz /swarm/assignments

# Check TF tree
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo robot/map robot/base_link
```
