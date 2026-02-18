# Key Concepts

## Map Merging (How Multiple Robot Maps Become One)

### The Problem
Each robot builds its own view of the world from its sensors. Robot 1 might see the north side, Robot 2 the south. Neither knows the full picture.

### The Solution: Map Fuser
The `swarm_map_fuser` node combines all robots' laser scans into a single global occupancy grid published on `/map`. This merged map covers the full 40x40m arena with 0.05m resolution.

### How It Works

1. **Individual Scans**: Each robot publishes its merged scan (`/scan_merged`, `/robot2/scan_merged`, etc.)
2. **Map Fuser**: Raytraces all scans into a shared occupancy grid
3. **Global Map**: Published on `/map` in the `robot/map` frame
4. **Shared Knowledge**: All robots (and the FrontierServer) use the same complete picture

### Why This Matters

Without merging, robots waste time re-exploring areas other robots already covered. With a shared map, the FrontierServer can direct robots to genuinely unexplored regions.

## ROS 2 Topics (The Communication System)

### What is a Topic?

Topics are publish/subscribe channels where ROS nodes share information:

```
Publisher Node → Topic → Subscriber Node(s)
    Robot       /scan      Map Fuser
    Map Fuser   /map       FrontierServer
```

### Topic Types in the Swarm System

#### Sensor Data
- `/robot/scan_merged`: Individual robot merged laser scans
- `/odom`, `/robot2/odom`: Robot odometry
- `/map`: Combined occupancy grid from all robots

#### Swarm Coordination
- `/swarm/frontiers`: Exploration frontier clusters (`PoseStamped`)
- `/swarm/assignments`: Drone-to-frontier assignments (`String`, format: `robot_id:frontier_id:cx:cy`)
- `/swarm/drone_states`: Drone status reports (`String`, format: `robot_id:state:x,y,yaw:assignment`)
- `/swarm/mission_state`: Mission phase (`String`)

#### Navigation
- Nav2 `NavigateToPose` action (per drone): Used by DroneExecutors to send goals
- `/cmd_vel`, `/robot2/cmd_vel`: Velocity commands from Nav2 to robots

### QoS (Quality of Service)

ROS 2 topics have reliability settings:
- **RELIABLE**: Guarantees message delivery (used for swarm coordination topics)
- **BEST_EFFORT**: Fast but may drop messages
- **TRANSIENT_LOCAL**: Keeps last message for new subscribers (used for `/map`)

## TF (Transform Frames)

### What is TF?

TF is ROS 2's coordinate system manager. It tracks relationships between different reference frames:

```
robot/map (fixed, global)
├── robot/odom → robot/base_footprint → robot/base_link
├── robot2/odom → robot2/base_footprint → robot2/base_link
├── robot3/odom → ...
├── robot4/odom → ...
└── robot5/odom → ...
```

### Why TF Matters

The FrontierServer publishes frontier coordinates in `robot/map` frame. DroneExecutors look up their own drone's position in that same frame via TF to calculate distances and navigation goals.

### TF Lookup Example

```python
transform = tf_buffer.lookup_transform('robot/map', f'{robot_name}/base_footprint', Time())
x = transform.transform.translation.x
y = transform.transform.translation.y
```

## SLAM (Mapping)

### What is SLAM?

**S**imultaneous **L**ocalization **A**nd **M**apping: the robot builds a map while figuring out its position on that map.

### In the Swarm Context

Instead of individual per-robot SLAM, this system uses a centralized `swarm_map_fuser` that raytraces all robots' scans into a single global occupancy grid. This is simpler than running N SLAM instances and avoids map alignment issues.

## Frontier Detection

### What is a Frontier?

A frontier is a free cell adjacent to unknown space on the occupancy grid. Clusters of frontier cells represent regions worth exploring.

### How FrontierServer Works

1. Scan the `/map` grid for frontier cells (free cells neighboring unknown cells)
2. Cluster connected frontier cells using flood-fill
3. Filter out clusters smaller than `min_frontier_cluster_size`
4. Compute each cluster's centroid as the exploration target
5. Publish centroids on `/swarm/frontiers`

## Goal Allocation

### Assignment Format

Assignments are published as strings with the format:
```
robot_id:frontier_id:cx:cy
```

For example: `robot:f_042:5.200:3.800` means "robot should navigate to frontier f_042 at coordinates (5.2, 3.8)".

### Greedy Scoring

The GoalAllocator scores each robot-frontier pair:
```
Score = (gain_weight × frontier_size) - (cost_weight × distance)
```

It greedily assigns the highest-scoring pair, removes both from consideration, and repeats.

## Nav2 Integration

### How Drones Navigate

Each DroneExecutor uses Nav2's `NavigateToPose` action client:

1. Receives an assignment with target coordinates `(cx, cy)`
2. Constructs a `PoseStamped` goal in the `robot/map` frame
3. Computes yaw to face the goal direction
4. Sends the goal to Nav2 via `NavigateToPose` action
5. Monitors for success, failure, or timeout
6. On success → `AVAILABLE` for next assignment
7. On failure → `RECOVERY` state with retry logic

### Shutdown Cleanup

DroneExecutors cancel any active Nav2 goals when the node is destroyed (e.g., on Ctrl+C), ensuring clean shutdown.

## Web Dashboard

### Architecture

The `swarm_dashboard` node runs both a ROS 2 subscriber and a Python stdlib HTTP server:

- **ROS 2 side**: Subscribes to `/swarm/drone_states`, `/swarm/frontiers`, `/swarm/assignments`, `/swarm/mission_state`
- **HTTP side**: Serves a single-page HTML dashboard with embedded CSS/JS on port 5000
- **SSE endpoint** (`/api/stream`): Pushes JSON state every 500ms to the browser
- **REST endpoint** (`/api/state`): One-shot JSON polling

No external dependencies (no Flask, no npm) -- uses Python's built-in `http.server` module.

### Dashboard Panels

- **Header**: Mission state, uptime, drone count, frontier count, goals assigned/reached/failed
- **Drone table**: Name, FSM state (color-coded), position, current assignment, time-in-state
- **Performance**: Publish rate stats (Hz per topic), goal completion counts
- **Frontier summary**: Active count, top 10 by size
- **Event log**: Last 50 events (state transitions, assignments, failures)

## Launch Files

### test_swarm.launch.py

The integration test launch file:
1. Includes `tof_slam_sim/swarm_fast.launch.py` for simulation infrastructure
2. Layers `swarm_control.launch.py` on top after an 8-second delay
3. Defaults to 5 robots with Nav2 enabled and distributed explorer disabled

### swarm_control.launch.py

Launches only the swarm control nodes:
- FrontierServer, GoalAllocator, MissionSupervisor, TrafficManager
- 5 DroneExecutor nodes (one per robot)
- SwarmDashboard (conditionally, via `dashboard` argument)

## Parameters

Parameters are configurable settings that nodes read at startup:

```python
self.declare_parameter('speed', 1.0)
speed = self.get_parameter('speed').value
```

Parameters can be set via:
1. **Code defaults**: `declare_parameter('speed', 1.0)`
2. **Launch files**: `parameters=[{'speed': 2.0}]`
3. **Runtime**: `ros2 param set /node speed 3.0`
