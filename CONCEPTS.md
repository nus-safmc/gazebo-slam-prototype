# Gazebo SLAM Prototype — Concepts & How It Fits Together

This repo is a ROS 2 + Gazebo simulation stack for fast 2D mapping and multi‑robot exploration using a quadcopter model instrumented with 8 simulated ToF sensors.

It contains two “tracks”:

- **PX4 SITL track** (more realistic autopilot integration; runs `PX4-Autopilot/` SITL).
- **Fast prototyping track** (lightweight `rex_quadcopter` model + ROS navigation/mapping tools; supports swarm runs up to 15 drones).

The rest of this document focuses on the **fast prototyping track**, since that’s what the swarm tooling uses.

## PX4 SITL Track (Single Drone)

If you want the PX4 SITL track (real autopilot in `PX4-Autopilot/`) instead of the lightweight `cmd_vel` model:

- Requires `MicroXRCEAgent` (PX4 uXRCE-DDS agent). If `pixi run -e jazzy px4_sitl` errors, install it (e.g. `sudo apt install micro-xrce-dds-agent`).
- PX4 + Gazebo + ToF → merged scan: `pixi run -e jazzy px4_sitl`
- PX4 + SLAM Toolbox: `pixi run -e jazzy px4_sitl_slam`
- PX4 + SLAM + Nav2 frontier exploration (PX4 offboard): `pixi run -e jazzy px4_nav2_explore`
- To capture a full terminal log while still seeing it live: `pixi run -e jazzy px4_nav2_explore_log` (writes to `log/run_logs/`).
- To tail the latest log: `pixi run -e jazzy tail_log -- px4_nav2_explore`

---

## What’s Used

**Environment / build**
- **Pixi + RoboStack**: reproducible ROS 2 environments (`pixi.toml`).
- **ROS 2 Jazzy**: middleware + tooling (`rclpy`, `nav2_*`, `slam_toolbox`, `rviz2`).

**Simulation**
- **Gazebo Harmonic** (`gz sim`): physics + sensors + world.
- **ros_gz_bridge** (`parameter_bridge`): bridges Gazebo topics ↔ ROS topics.

**Mapping & exploration**
- **8× ToF ring**: simulated as 8 short‑range scan sensors around the drone.
- **`scan_merger`**: merges 8 directional scans into a single 360° `/scan_merged`.
- **`swarm_map_fuser`**: builds a global `/map` by ray‑tracing robot scans using (sim) odometry/TF.
- **Nav2** (`nav2_bringup`): local + global planning on the fused map.
- **`nav2_frontier_explorer`**: picks frontier goals and drives Nav2 using `NavigateToPose`.
- **`auto_pilot` (explore mode)**: lighter alternative to Nav2 for large swarms.

**UIs**
- **Spawn selector UI**: click to place each robot; writes a temporary world SDF for Gazebo.
- **Health dashboard UI**: runtime status + publishes RViz markers for robot positions.

---

## Core Ideas / Architecture

### 1) World + robot models

- Worlds are SDF files under `tof_slam_sim/worlds/`:
  - `playfield_sparse.sdf`: open arena (good for single robot / quick tests).
  - `playfield_swarm.sdf`: swarm‑friendly arena layout + default robot includes.
  - World size is currently **~40×40 m** (ground plane size is `40 40`).

- Robots are Gazebo models under `tof_slam_sim/models/`:
  - `rex_quadcopter`: base model (robot “1”).
  - `rex_quadcopter_2 … rex_quadcopter_15`: variants with unique `cmd_vel` topics + TF frame IDs so multiple robots don’t collide on topics/frames.

### 2) Gazebo ↔ ROS bridging

`tof_slam_sim/launch/sim_with_bridge.launch.py`:
- Starts Gazebo (`gz sim`) with the chosen world.
- Generates a YAML bridge config at runtime for a list of robots (e.g. `robot,robot2,...`).
- Starts a single `ros_gz_bridge parameter_bridge` process that bridges:
  - `/clock` (Gazebo → ROS)
  - `/model/<robot>/odometry` (Gazebo → ROS `/odom` or `/<robot>/odom`)
  - the 8 sensor scan topics (Gazebo → ROS `/<robot>/scan/<sensor>`)
  - `cmd_vel` (ROS → Gazebo; per‑robot `cmd_vel` topics map to per‑robot Gazebo topics)

### 3) Sensor pipeline (8 ToF sensors → one 360° scan)

Each robot publishes 8 scan topics (one per sensor):
- `/<robot>/scan/front`
- `/<robot>/scan/front_right`
- `/<robot>/scan/right`
- … etc

`tof_slam_sim/tof_slam_sim/scan_merger.py` merges these into:
- `/scan_merged` for `robot`
- `/<robotN>/scan_merged` for others

### 4) Swarm mapping (fast global map fusion)

`tof_slam_sim/tof_slam_sim/swarm_map_fuser.py` creates a single global occupancy grid:
- Subscribes to each robot’s `scan_merged`
- Uses TF (`robot/map` → `<robot>/base_footprint`) to ray‑trace beams
- Publishes:
  - `/map` (OccupancyGrid)
  - `/map_updates` (OccupancyGridUpdate)

This is **not SLAM**: it assumes simulated odometry/TF is accurate enough and prioritizes speed + stability for multi‑robot runs.

### 5) Exploration / navigation

`tof_slam_sim/launch/swarm_fast.launch.py` can run either:

- **Nav2 mode**:
  - Starts a Nav2 stack per robot (namespaced) using `nav2_bringup`
  - Starts `nav2_frontier_explorer` per robot to generate goals

- **Autopilot mode**:
  - Runs the repo’s `auto_pilot` node per robot (lighter; good for many drones)

Both modes enforce arena bounds so robots don’t “plan” into unknown space outside the perimeter.

### 6) Runtime monitoring + RViz robot locations

`tof_slam_sim/tof_slam_sim/drone_health_dashboard.py`:
- Subscribes to `/<robot>/odom`, `/<robot>/cmd_vel`, and `/<robot>/scan_merged`
- Classifies each robot as:
  - `MOVING`, `STATIONARY`, `STUCK`, `CRASHED`, `NO_ODOM`
- Publishes RViz markers:
  - `/swarm/drone_markers` (MarkerArray: arrows + labels + trails)
  - `/swarm/drone_poses` (PoseArray: simple pose list)

The default RViz config (`tof_slam_sim/config/slam.rviz`) includes a `MarkerArray` display for `/swarm/drone_markers`.

---

## How To Run (Fast Track)

### Build

```bash
pixi run -e jazzy build
```

### Clean up stuck Gazebo / bridge processes

```bash
pixi run -e jazzy cleanup_sim
```

### Swarm run (recommended entry point)

Spawn UI (click placements):

```bash
pixi run -e jazzy swarm_fast -- --num-drones 6
```

Skip UI and use default spawns:

```bash
pixi run -e jazzy swarm_fast -- --default --num-drones 6
```

Equivalent direct launch (no wrapper):

```bash
ros2 launch tof_slam_sim swarm_fast.launch.py num_robots:=6 default_spawn:=true
```

Useful launch flags:
- `health_ui:=false` (disable the Tk dashboard window)
- `publish_drone_markers:=false` (disable RViz marker publishing)
- `rviz:=false` (don’t start RViz)
- `run_autopilot:=true run_nav2:=false run_explorer:=false` (lighter exploration for big swarms)

### Single robot run

```bash
pixi run -e jazzy old_sim
```

Or directly:

```bash
ros2 launch tof_slam_sim sim_with_bridge.launch.py world:=playfield_sparse.sdf robots:=robot
```

---

## Where To Look in the Code

- Launch:
  - `tof_slam_sim/launch/sim_with_bridge.launch.py` (Gazebo + bridge)
  - `tof_slam_sim/launch/swarm_fast.launch.py` (swarm stack: spawn UI, mergers, fuser, Nav2/autopilot, RViz)
- Core nodes:
  - `tof_slam_sim/tof_slam_sim/scan_merger.py`
  - `tof_slam_sim/tof_slam_sim/swarm_map_fuser.py`
  - `tof_slam_sim/tof_slam_sim/nav2_frontier_explorer.py`
  - `tof_slam_sim/tof_slam_sim/auto_pilot_node.py`
  - `tof_slam_sim/tof_slam_sim/drone_health_dashboard.py`
- Worlds / models:
  - `tof_slam_sim/worlds/`
  - `tof_slam_sim/models/`
