# Quick Start Guide

Get your swarm control system running in 5 minutes.

## Prerequisites

- ROS 2 Jazzy installed via RoboStack PIXI
- Gazebo simulation (included in `tof_slam_sim`)
- Project built with `pixi run -e jazzy build`

## Step 1: Build the System

```bash
pixi run -e jazzy build
```

## Step 2: Launch the Full Swarm Test

```bash
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py
```

This starts:
- Gazebo with 5 robots in the `playfield_swarm.sdf` world
- ROS-Gazebo bridges for all sensors and actuators
- TF broadcaster, scan mergers, and map fuser
- 5 Nav2 stacks (one per robot)
- All swarm control nodes (FrontierServer, GoalAllocator, MissionSupervisor, TrafficManager)
- 5 DroneExecutor nodes (one per robot)
- Web dashboard

## Step 3: Open the Web Dashboard

Navigate to **http://localhost:8080** in your browser.

The dashboard shows:
- **Header bar**: Mission state, uptime, total goals assigned/reached
- **Drone table**: Name, FSM state (color-coded), position, current assignment, time-in-state
- **Performance panel**: Publish rates (drone state Hz, frontier Hz, assignment Hz), goal counts
- **Frontier summary**: Count of active frontiers, top 10 by size
- **Event log**: Scrolling list of recent state transitions, assignments, and failures

## Step 4: Monitor Topics (Optional)

Open separate terminals:

```bash
# Watch frontiers being found
pixi run -e jazzy ros2 topic echo /swarm/frontiers

# Watch assignments (format: robot_id:frontier_id:cx:cy)
pixi run -e jazzy ros2 topic echo /swarm/assignments

# Watch robot states
pixi run -e jazzy ros2 topic echo /swarm/drone_states

# Watch mission state
pixi run -e jazzy ros2 topic echo /swarm/mission_state
```

## What You Should See

### In the Dashboard:
- Drones progressing through states: `BOOT` -> `PREFLIGHT` -> `ARMED` -> `STAGING` -> `AVAILABLE`
- Frontier count increasing as the map builds
- Assignments appearing as drones become `AVAILABLE`
- Drones entering `EXECUTING_GOAL` and navigating to frontiers
- Performance metrics showing healthy publish rates

### In the Logs:
```
[frontier_server]: Found 5 frontiers, total size: 127 cells
[goal_allocator]: Assigned robot -> f_042 at (5.2, 3.8)
[robot_executor]: AVAILABLE -> EXECUTING_GOAL
[mission_supervisor]: Mission: IDLE -> READY
```

### Robot Behavior:
- Robots take off in two groups (takeoff windows A and B)
- Each robot gets assigned to explore different frontiers
- Map grows as robots explore
- Mission completes when all areas explored

## Common First Issues

### "No frontiers found"
- Wait for robots to start scanning -- the map needs data before frontiers appear
- Check the `/map` topic is publishing: `ros2 topic hz /map`

### "Drones stuck in BOOT"
- Nav2 may still be initializing (can take 30-60 seconds)
- Check Nav2 nodes: `ros2 node list | grep controller_server`

### "Dashboard not loading"
- Check port 8080 is not in use
- Look for `swarm_dashboard` in `ros2 node list`

### "Robots not moving"
- Verify Nav2 is running: `ros2 node list | grep nav2`
- Check TF is working: `ros2 run tf2_ros tf2_echo robot/map robot/base_link`

## Stopping the System

Press `Ctrl+C` in the terminal running the launch file. DroneExecutors cancel any active Nav2 goals on shutdown.

If processes linger:
```bash
pkill -f ros2
pkill -f gz
```

## Success Metrics

**Good signs:**
- Frontiers published regularly (>0.5 Hz)
- Assignments happening as drones become available
- Drones cycling: `AVAILABLE` -> `EXECUTING_GOAL` -> `AVAILABLE`
- Map coverage growing over time
- Dashboard showing healthy rates (drone state >1 Hz)
