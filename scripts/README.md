# Scripts Organization

This directory contains scripts for managing the Gazebo SLAM prototype simulation system. Scripts are organized by function for better maintainability.

## Directory Structure

### `core/` - Core Infrastructure
Essential scripts for running and monitoring simulations.

- `cleanup_sim.sh` - Reset simulation state by terminating all processes
- `run_with_log.sh` - Run commands with timestamped logging and filtering
- `tail_latest_log.sh` - Monitor the most recent log files

### `px4/` - PX4 Integration
Scripts for PX4 Software-In-The-Loop (SITL) integration.

- `activate-cyclonedds.sh` - Configure ROS to use CycloneDDS
- `activate-cyclonedds.bat` - Windows version of CycloneDDS activation
- `cyclonedds.xml` - CycloneDDS configuration file
- `check_microxrce_agent.sh` - Verify MicroXRCE-DDS Agent installation
- `build_microxrce_agent.sh` - Build MicroXRCE-DDS Agent from source
- `run_px4_sitl.sh` - Launch basic PX4 SITL simulation
- `run_px4_sitl_slam.sh` - Launch PX4 SITL with SLAM
- `run_px4_nav2_explore.sh` - Launch PX4 SITL with autonomous exploration

### `swarm/` - Multi-Robot Coordination
Scripts for managing multiple robot simulations.

- `run_swarm_fast.sh` - Launch multi-drone swarm (Gazebo-native)
- `run_px4_swarm_fast.sh` - Launch multi-drone swarm (PX4 SITL)

> **Note**: For advanced swarm coordination with centralized control, consider using the `swarm_control` package launch files (`test_swarm.launch.py`, `px4_test_swarm.launch.py`) which provide more sophisticated mission management and coordination features.

### `verify/` - System Health Checks
Scripts for validating system functionality.

- `verify_tf_rates.sh` - Check ROS TF tree and topic publication rates
- `verify_map_runtime.sh` - Validate map publishing and SLAM functionality

### `test/` - Automated Testing
Scripts for automated testing and validation.

- `px4_map_smoketest.sh` - Test single-drone map building
- `px4_swarm_smoketest.sh` - Test multi-drone swarm functionality

## Quick Start Commands

```bash
# Clean start a simulation
pixi run -e jazzy bash scripts/core/cleanup_sim.sh
pixi run -e jazzy bash scripts/core/run_with_log.sh sim_session ros2 launch tof_slam_sim sim_with_bridge.launch.py

# Monitor logs
pixi run -e jazzy bash scripts/core/tail_latest_log.sh sim_session

# Verify system health
pixi run -e jazzy bash scripts/verify/verify_tf_rates.sh

# Run PX4 simulation
pixi run -e jazzy bash scripts/px4/check_microxrce_agent.sh
pixi run -e jazzy bash scripts/px4/run_px4_sitl_slam.sh

# Run swarm testing
pixi run -e jazzy bash scripts/test/px4_swarm_smoketest.sh --num-drones 3
```

## Backward Compatibility

All scripts maintain backward compatibility - you can still call them from the root `scripts/` directory. The root directory contains wrapper scripts that delegate to the organized versions.

## Development Guidelines

- **New scripts**: Add to the appropriate subdirectory based on function
- **Modifications**: Update both the organized version and the wrapper
- **Documentation**: Update this README when adding new scripts
- **Testing**: Test both the organized path and wrapper path

## Common Patterns

### Logging
Most simulation launches should use `run_with_log.sh` for consistent logging:

```bash
pixi run -e jazzy bash scripts/core/run_with_log.sh experiment_name command args...
```

### PX4 Workflows
PX4 scripts automatically handle DDS configuration and cleanup:

```bash
# Single drone with SLAM
pixi run -e jazzy bash scripts/px4/run_px4_sitl_slam.sh

# Multi-drone swarm
pixi run -e jazzy bash scripts/test/px4_swarm_smoketest.sh --num-drones 5
```

### Health Monitoring
Use verification scripts during development:

```bash
# Check TF tree and topic rates
pixi run -e jazzy bash scripts/verify/verify_tf_rates.sh

# Validate map publishing
pixi run -e jazzy bash scripts/verify/verify_map_runtime.sh
```