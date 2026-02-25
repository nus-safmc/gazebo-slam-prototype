# Configuration Files

## SLAM Toolbox

### slam_toolbox_px4_fast.yaml

Fast mapping profile for PX4 drones.  Key characteristics:
- 0.10 m resolution (coarser for speed)
- No loop closing
- Zero minimum travel distance (allows map init while hovering)
- Uses `robot/odom`, `robot/map`, `robot/base_footprint` frames

### slam_toolbox_px4_robust.yaml

Higher-quality mapping profile.  Key characteristics:
- 0.05 m resolution
- Scan matching disabled (relies on PX4 EKF2 + external vision)
- Minimum travel thresholds to avoid duplicate scans while hovering
- Speckle suppression (`min_pass_through: 4`, `occupancy_threshold: 0.25`)

### slam_toolbox_fast.yaml / slam_toolbox.yaml

Legacy SLAM configs for the Gazebo-only rex_quadcopter setup.

## Nav2

### nav2_params_rex.yaml

Full Nav2 stack parameters: DWB local planner (holonomic, 0.5 m/s max),
SmacPlanner2D global planner, collision monitor with footprint approach,
local costmap (7 x 7 m rolling window, 0.10 m resolution), global costmap
with inflation layers.  All frames use `robot/` prefix.

## RViz

### slam_px4.rviz

RViz configuration for PX4 simulation with:
- `/scan_merged_viz` LaserScan display
- `/map` OccupancyGrid
- `/swarm/drone_markers` MarkerArray
- Fixed frame: `robot/map`

### slam.rviz

Legacy RViz configuration.

## PX4 Parameters

### px4-rc.params (in scripts/)

PX4 SITL startup parameter overrides sourced by `rcS`.  Critical settings:
- `EKF2_EV_CTRL 13` -- fuse external vision horizontal pos + velocity + yaw
- `EKF2_HGT_REF 0` -- barometer as primary height reference
- `NAV_DLL_ACT 0` -- don't block arming for missing GCS link
- `SYS_HAS_MAG 0` -- no magnetometer required (indoor)
