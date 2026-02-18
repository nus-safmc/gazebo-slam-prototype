# Swarm Control Documentation Index

## Documentation Overview

This documentation explains the centralized swarm control system for multi-robot exploration using 5 drones with Nav2 navigation and a live web dashboard.

## Documentation Structure

### Core Documentation
- **[README.md](README.md)** -- Main overview, key concepts, quick start
- **[quick_start.md](quick_start.md)** -- 5-minute setup guide
- **[troubleshooting.md](troubleshooting.md)** -- Common issues and solutions

### Component Documentation
- **[frontier_server.md](frontier_server.md)** -- Centralized frontier detection
- **[goal_allocator.md](goal_allocator.md)** -- Smart task assignment
- **[drone_executor.md](drone_executor.md)** -- Per-robot Nav2 control FSM
- **[mission_supervisor.md](mission_supervisor.md)** -- Global mission coordination
- **[traffic_manager.md](traffic_manager.md)** -- Collision avoidance / proximity monitoring

### Technical Concepts
- **[concepts.md](concepts.md)** -- Map merging, ROS topics, TF, SLAM, parameters

### Diagrams
- **[diagrams/architecture.puml](diagrams/architecture.puml)** -- System architecture
- **[diagrams/drone_executor_states.puml](diagrams/drone_executor_states.puml)** -- Robot state machine
- **[diagrams/mission_supervisor_states.puml](diagrams/mission_supervisor_states.puml)** -- Mission phases
- **[diagrams/data_flow.puml](diagrams/data_flow.puml)** -- Information flow

## Reading Guide

### New to Everything?
1. Start with **[README.md](README.md)** -- Learn basic concepts
2. Try **[quick_start.md](quick_start.md)** -- Get it running
3. Read **[concepts.md](concepts.md)** -- Understand key ideas
4. Debug with **[troubleshooting.md](troubleshooting.md)**

### Know ROS, New to Swarm?
1. **[README.md](README.md)** -- System overview
2. **[quick_start.md](quick_start.md)** -- Hands-on start
3. Individual component docs for deep dives
4. **[concepts.md](concepts.md)** -- Swarm-specific concepts

### Developer/Contributor?
1. **[README.md](README.md)** -- Architecture overview
2. Component documentation for implementation details
3. **[troubleshooting.md](troubleshooting.md)** -- Debug techniques
4. Study the PlantUML diagrams

## Key Topics Covered

### System Architecture
- Centralized control with Nav2 integration
- ROS 2 node communication
- State machines and FSMs
- Web dashboard via SSE

### Technical Concepts
- Map merging and SLAM
- Frontier detection algorithms
- Goal assignment optimization (greedy scoring)
- Real-time monitoring via web dashboard

### Components
- FrontierServer: Exploration opportunity detection from `/map`
- GoalAllocator: Optimal task distribution (format: `robot_id:frontier_id:cx:cy`)
- DroneExecutor: Individual robot control via Nav2 `NavigateToPose`
- MissionSupervisor: Global coordination with takeoff windows
- TrafficManager: Proximity monitoring
- SwarmDashboard: Web UI on `http://localhost:5000`

## File Organization

```
docs/
├── README.md              # Main overview
├── quick_start.md         # Fast setup guide
├── troubleshooting.md     # Problem solving
├── concepts.md            # Technical concepts
├── frontier_server.md     # Component docs
├── goal_allocator.md
├── drone_executor.md
├── mission_supervisor.md
├── traffic_manager.md
├── index.md               # This file
└── diagrams/              # PlantUML diagrams
    ├── architecture.puml
    ├── drone_executor_states.puml
    ├── mission_supervisor_states.puml
    └── data_flow.puml
```

## Related Documentation

### External Resources
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Nav2 Documentation](https://docs.nav2.org/)

### Project Resources
- `../tof_slam_sim/README.md` -- Simulation package docs
- `../pixi.toml` -- Environment setup
