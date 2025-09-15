# 🛰️ 8× VL53L7CX ToF Sensor Simulation for SLAM Validation
**Simulation project for validating 2D SLAM using an array of 8 simulated ToF sensors (VL53L7CX-equivalent) in Gazebo Harmonic.**

---

## 📌 Objective

Build a simulation environment in **Gazebo Harmonic** to emulate a **ring of eight VL53L7CX sensors** mounted on a robot (e.g. drone or rover), each providing a **8×8 depth grid** with:
- **Horizontal FOV:** ~60°
- **Vertical FOV:** ~45° (will be collapsed)
- **Range:** 0.02 m to 3.5 m
- **Output Resolution:** 8×8 depth points

The simulation will be used to:
- Validate **2D SLAM** pipelines.
- Test **sensor coverage, noise, and drift** in controlled environments.
- Provide a bridge from simulated depth to `LaserScan` topics usable by standard SLAM stacks.

---

## 🏗️ Project Architecture Overview

### 1. Sensor Emulation (Gazebo Side)
- Emulate each sensor using a **`<depth_camera>` sensor** in SDF.
- Configure each sensor for:
  - 8×8 resolution
  - 60° horizontal FOV
  - 0.02–3.5 m depth clipping
- Mount 8 such sensors around the robot body at 45° intervals to cover 360°.

**Key Outputs:**
- `/vl53_X/depth` → `gz.msgs.Image` (float32 depth map)
- `/vl53_X/camera_info` → `gz.msgs.CameraInfo`

### 2. ROS 2 Integration (Bridge Layer)
- Use bridege to forward Gazebo sensor data to ROS 2.
- Bridge each sensor’s depth image + camera info into ROS topics:
  - `/vl53_X/depth` → `sensor_msgs/Image`
  - `/vl53_X/camera_info` → `sensor_msgs/CameraInfo`

This enables downstream ROS nodes to process the sensor data in real time.

### 3. Depth → Laser Projection (ROS 2 Node)
- Custom ROS 2 node that:
  - Subscribes to each 8×8 depth image
  - **Collapses the vertical axis** → one horizontal ring of 8 depth points
  - Outputs `sensor_msgs/LaserScan` with:
    - `angle_min = -HFOV/2`, `angle_max = +HFOV/2`
    - `angle_increment = HFOV/8`
    - `range_min = 0.02`, `range_max = 3.5`
- Publish as `/scan/front`, `/scan/front_left`, etc.

### 4. Scan Merging
- Combine 8 per-sensor `LaserScan` messages into a **single 360° scan**.
- Merge by rotating each scan based on its mounting yaw and concatenating them in angular order.

**Output:** `/scan_merged` → complete 360° scan

### 5. SLAM Validation
- Feed `/scan_merged` into `slam_toolbox` (or other 2D SLAM system).
- Drive the robot around test worlds (rooms, corridors, obstacles).
- Verify:
  - Map closure and accuracy
  - Loop closure behaviour
  - Drift over distance

---

## 🧠 Key Assumptions & Design Choices
- **Depth Camera Emulation:** Using Gazebo’s native `depth_camera` is simpler and more stable than writing a custom ToF sensor plugin, while matching the VL53L7CX’s functional behaviour.
- **Vertical Compression:** The real VL53L7CX has 8×8 zones; collapsing vertical depth gives a single horizontal slice usable for 2D SLAM.
- **360° Coverage:** 8 sensors spaced at 45° offsets provide nearly seamless coverage for robust SLAM performance.
- **Noise-Free Initial Validation:** Initial simulation will use ideal (noise-free) depth; noise models can be added later if needed.

---

## 📂 Proposed Project Structure

