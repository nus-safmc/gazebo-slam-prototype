# Swarm Control Documentation

Welcome! This documentation explains the centralized swarm control system for multi-robot exploration.

## What is This System?

Multiple drones (robots) explore an unknown area together. The **centralized** approach has one "brain" that makes smart decisions for all drones:

- One frontier scan finds all exploration opportunities from the merged map
- Smart assignment of goals to drones via a greedy scoring algorithm
- Per-drone FSMs execute goals through Nav2 `NavigateToPose`
- A live web dashboard at `http://localhost:5000` monitors everything in real time

## Key Concepts

### ROS 2
ROS 2 is the robotics middleware -- a set of tools that help robots communicate and work together via topics, services, and actions.

### Node
A ROS 2 node is a single process that does one specific job and communicates with other nodes.

### Topic
Topics are publish/subscribe channels where nodes share information.

### TF (Transform Frames)
TF tracks where everything is in 3D space, relating coordinate frames like `robot/map`, `robot/odom`, and `robot/base_link`.

### SLAM & Map Merging
Each robot's laser scans are merged into a single global occupancy grid (`/map`) by the `swarm_map_fuser` node. The `FrontierServer` detects unexplored regions from this combined map.

### Frontier
A frontier is the boundary between explored (known) and unexplored (unknown) areas on the map. Robots are sent to frontiers to expand the known area.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WORLD (Gazebo Simulation)                │
│  5 robots with 8 ToF sensors each, spawned inside the      │
│  (-9.4, 9.4) exploration area                              │
└────────────┬──────────────┬────────────────────────────────┘
             │              │
             ▼              │
┌─────────────────────────────────────────────────────────────┐
│                ROS 2 MIDDLEWARE                             │
│  /scan_merged (per robot), /odom, /cmd_vel, /map, TF       │
└────────────┬──────────────┬────────────────────────────────┘
             │              │
             ▼              │
┌─────────────────────────────────────────────────────────────┐
│                SWARM CONTROL SYSTEM                        │
│  Central: FrontierServer, GoalAllocator,                   │
│           MissionSupervisor, TrafficManager                │
│  Per-robot: DroneExecutor (×5) → Nav2 NavigateToPose       │
│  Dashboard: SwarmDashboard (web UI on :5000)               │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Sensors** -- Robots detect the world with 8 ToF depth cameras each
2. **Scan merger** -- Individual scans become a merged LaserScan per robot
3. **Map fuser** -- All robots' scans combine into one global `/map`
4. **FrontierServer** -- Finds frontier clusters from the map
5. **GoalAllocator** -- Assigns frontiers to available drones (greedy scoring)
6. **DroneExecutors** -- Send `NavigateToPose` goals to Nav2 for each drone
7. **MissionSupervisor** -- Coordinates takeoff windows and mission phases
8. **SwarmDashboard** -- Streams all telemetry to a web UI via SSE

## Package Structure

```
swarm_control/
├── package.xml              # Package description and dependencies
├── CMakeLists.txt           # Build configuration
├── setup.py                 # Python package setup
├── swarm_control/           # Main Python package
│   ├── __init__.py
│   ├── frontier_server.py   # Frontier detection from /map
│   ├── goal_allocator.py    # Greedy frontier-to-drone assignment
│   ├── drone_executor.py    # Per-robot FSM with Nav2 integration
│   ├── mission_supervisor.py # Global mission FSM
│   ├── traffic_manager.py   # Proximity monitoring
│   └── swarm_dashboard.py   # Web dashboard (Flask-free, stdlib HTTP + SSE)
├── launch/
│   ├── swarm_control.launch.py  # Swarm control nodes only
│   └── test_swarm.launch.py    # Full sim + swarm control test
├── config/                  # Configuration files
└── docs/                    # Documentation (this folder)
```

## Quick Start

```bash
# 1. Build everything
pixi run -e jazzy build

# 2. Launch full simulation with 5 drones + swarm control
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py

# 3. Open the web dashboard
# Navigate to http://localhost:5000

# 4. Watch topics in separate terminals
pixi run -e jazzy ros2 topic echo /swarm/frontiers
pixi run -e jazzy ros2 topic echo /swarm/assignments
```

## Detailed Documentation

Continue reading for detailed explanations of each component:

- [Quick Start](quick_start.md) -- 5-minute setup guide
- [Concepts](concepts.md) -- Map merging, TF, SLAM, parameters
- [FrontierServer](frontier_server.md) -- Frontier detection
- [GoalAllocator](goal_allocator.md) -- Smart task assignment
- [DroneExecutor](drone_executor.md) -- Per-robot control FSM
- [MissionSupervisor](mission_supervisor.md) -- Global mission coordination
- [TrafficManager](traffic_manager.md) -- Collision avoidance
- [Troubleshooting](troubleshooting.md) -- Common issues and solutions
- [Index](index.md) -- Complete documentation index
