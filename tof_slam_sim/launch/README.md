# Launch Files

## PX4 Launch Files (recommended)

### px4_sitl.launch.py

Base PX4 SITL launch for a **single drone**.  Starts Gazebo server, one PX4
SITL instance, MicroXRCE-DDS agent, `ros_gz_bridge` for depth topics and
clock, and the `tof_to_scan` sensor processing node.

Key arguments: `world`, `gz_gui`, `px4_dir`, `run_gz`, `run_px4`,
`run_bridge`, `run_agent`, `use_visual_odometry`, `px4_model_pose`.

### px4_sitl_slam.launch.py

Single-drone PX4 with SLAM.  Includes `px4_sitl.launch.py` and adds:
- `px4_vehicle_odometry_to_odom` (PX4 NED to ROS ENU odometry)
- `odom_tf_publisher` (publish odometry TF)
- `slam_toolbox` (sync lifecycle node)
- `rviz2`

Useful for development and parameter tuning.

### px4_swarm_fast.launch.py

**Multi-drone** PX4 SITL infrastructure.  Spawns N drones each with:
- Dedicated PX4 SITL instance (unique ports)
- `gz_pose_info_to_pose_stamped` (Gazebo native pose bridge)
- `pose_to_px4_visual_odometry` (ground-truth to EKF2)
- `px4_vehicle_odometry_to_odom` (PX4 odometry to ROS)
- 8 x `tof8x8_to_scan` (depth image to LaserScan)
- `scan_merger` (8 scans to 360-degree scan)
- `twist_to_px4_offboard` (cmd_vel to PX4 offboard)

Plus shared nodes: `swarm_tf_broadcaster`, `swarm_map_fuser`,
`drone_health_dashboard`, optional `rviz2`.

Key arguments: `num_robots`, `robots`, `world`, `default_spawn`,
`gz_gui`, `target_alt_m`, `max_alt_m`, `run_autopilot`.

### px4_nav2_explore.launch.py

Single-drone PX4 with Nav2 autonomous frontier exploration.
Full navigation stack including Nav2 bringup, costmaps, and
the `nav2_frontier_explorer` node.

## Legacy Launch Files (deprecated)

### swarm_fast.launch.py

Multi-drone Gazebo-only simulation using `rex_quadcopter` models.
**Deprecated** -- use `px4_swarm_fast.launch.py` instead.

### sim_with_bridge.launch.py

Single / multi-drone Gazebo infrastructure with `ros_gz_bridge`.
**Deprecated** -- use `px4_sitl.launch.py` instead.

### slam_fast.launch.py / slam_test.launch.py

Legacy SLAM test launch files.

## Integration with swarm_control

The recommended way to run the full stack is via `swarm_control`:

```bash
ros2 launch swarm_control test_swarm.launch.py num_robots:=4
```

This includes `px4_swarm_fast.launch.py` for infrastructure and layers
the centralized swarm_control nodes (FrontierServer, GoalAllocator,
DroneExecutors, etc.) on top.
