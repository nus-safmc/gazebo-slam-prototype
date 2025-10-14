
# 🛰️ 8× VL53L7CX ToF Drone Simulation in Gazebo Harmonic

**Objective:**  
Create a high-fidelity simulation in Gazebo Harmonic of a 15 cm quadcopter carrying 8 × VL53L7CX ToF sensors mounted in a circular ring, flying inside a 20 m × 20 m competition play field, to validate 2D SLAM performance.

---

## 📐 Physical Environment Specification

| Element               | Dimensions                    | Notes                                                             |
|------------------------|-------------------------------|------------------------------------------------------------------|
| Field Size             | 20 m × 20 m                   | Entire simulation world boundary                                 |
| Start Area             | 20 m × 6 m                    | At bottom edge of field (green zone)                             |
| Known Search Area      | 20 m × 14 m                   | Above start area (grey zone)                                    |
| Unknown Search Area    | 8 m × 8 m                     | Center of the field (yellow zone, partially walled)              |
| Perimeter Wall          | 1.5 m tall, ~0.2 m thick      | Surrounds field on 3 sides, with netting on top                   |
| Inner Walls             | 2.0 m tall, ~0.2 m thick      | Maze-like obstacles; minimum 2 m gap between them                 |
| Pillar Obstacles        | 0.3 m diameter, 2.0 m tall     | Each has 0.5 m base (0.15 m tall); at least 1 m from each other/walls |
| Danger Zones             | 1.5 m radius discs            | Non-traversable, marked in red                                   |
| Victim Markers           | Regular (blue triangle), bonus (red triangle) | Placed on floor or walls, used as visual landmarks |

---

**Important Constraints:**
- At least 2 m gap between inner walls
- At least 1 m gap between pillars and any walls/pillars
- All objects must have collision meshes for realistic navigation

**Gazebo Implementation Tips:**
- Use `box` geometries for walls, scaled appropriately
- Use `cylinder` geometries for pillars
- Place walls and pillars according to the provided layout image
- Assign unique names to each obstacle for easier debugging

---

## 🤖 Drone Specification

| Property           | Value              |
|---------------------|-------------------|
| Frame               | 15 cm × 15 cm X-quad |
| Height               | 5 cm body height |
| Sensor Mounting      | 8 × VL53L7CX arranged in a ring under the body |
| Mass (for physics)    | ~0.3 kg            |
| Propellers            | 4 visual-only (no thrust needed unless you want flight dynamics) |

**Simulation Note:**  
The drone will be spawned as a ground-hovering robot first (no flight dynamics) just to validate SLAM with sensor data. You can later add plugins (like `multicopter_motor_model`) if you want flight control.

---

## 📡 Sensor Specification

| Parameter         | Value            |
|-------------------|------------------|
| Sensor Type         | Depth Camera (8×8 grid) |
| Horizontal FOV       | 60° (1.047 rad) |
| Vertical FOV         | 45° (0.785 rad) |
| Range                | 0.02–3.5 m |
| Placement             | 8 sensors at 45° yaw offsets around bottom ring |

**Implementation Plan:**
- Define one `vl53l7cx.sdf` sensor model (depth_camera)
- Include 8 instances of it in the `quadcopter.sdf` model, positioned around the body

---

## ⚙️ Functional Software Pipeline

### Gazebo (Simulation)
- Launches the world with:
  - 20×20 m arena with start/known/unknown zones
  - Inner walls (2 m)
  - Perimeter walls (1.5 m)
  - Pillars (0.3 m Ø, 2 m height)
  - Danger zones, victims (optional visuals)
- Spawns quadcopter model with 8 ToF sensors

### ROS 2 Bridge
- `ros_gz_bridge` relays each sensor’s:
  - `/depth` (`sensor_msgs/Image`)
  - `/camera_info` (`sensor_msgs/CameraInfo`)

### Depth → LaserScan Node
- For each sensor:
  - Collapse 8×8 depth vertically to 8 horizontal beams
  - Create `sensor_msgs/LaserScan` message
  - Adjust `angle_min/max` based on sensor mounting yaw
- Merge all 8 scans into one `/scan_merged`

### SLAM
- Run `slam_toolbox` using `/scan_merged`
- Evaluate map quality and drift

---

## 📂 Suggested Project Structure

```

tof\_slam\_sim/
├── models/
│   ├── vl53l7cx/             # depth\_camera model
│   │   └── model.sdf
│   ├── quadcopter/
│   │   └── model.sdf         # 15cm body + 8 sensors
│   ├── inner\_wall.sdf
│   ├── pillar.sdf
│   └── perimeter\_wall.sdf
├── worlds/
│   └── playfield.world       # complete 20x20m field layout
├── launch/
│   ├── sim\_with\_bridge.launch.py
│   └── slam\_test.launch.py
├── src/
│   ├── tof8x8\_to\_scan.py     # depth to scan
│   ├── scan\_merger.py
│   └── test\_controller.py
├── config/
│   └── slam\_toolbox.yaml
└── README.md

```

---

## 📋 Development Checklist

**Phase 1 — Base Environment**
- [ ] Build 20×20 m playfield with zones, walls, pillars
- [ ] Add correct heights and gaps
- [ ] Confirm collisions work

**Phase 2 — Drone + Sensors**
- [ ] Build 15×15 cm quadcopter model
- [ ] Mount 8 depth cameras in a ring
- [ ] Verify sensor fields don’t overlap the body

**Phase 3 — ROS Integration**
- [ ] Bridge all 8 depth topics
- [ ] Visualize depth in RViz
- [ ] Write `tof8x8_to_scan.py`

**Phase 4 — SLAM Validation**
- [ ] Merge scans
- [ ] Run slam_toolbox
- [ ] Evaluate map closure accuracy

---

## ⚠️ Notes
- Vertical size differences are large (15 cm drone vs 2 m walls) — ensure sensors are mounted low enough to “see” the walls and pillars at their bases.
- Use `static=true` for walls and pillars to save compute.
- Keep all positions in meters (Gazebo default).
- You can start without flight physics — just move the robot using `teleport` or a diff-drive plugin.

---

## 🧭 Future Work
- Add IMU/odometry noise
- Add visual victim detection
- Add real flight dynamics plugins
- Export maps for real-world navigation
