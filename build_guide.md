# Gazebo SLAM Build Guide

This guide explains how to build and manage your ROS 2 packages in the RoboStack + Pixi environment. 

## Overview: How Building Works in Your Project

Your project uses a modern approach called **RoboStack** that integrates ROS 2 with **Pixi** (a package manager). Instead of the traditional ROS workspace approach, your packages are built directly into isolated environments.

### Key Concept: Everything Lives in Pixi Environments

```
Traditional ROS:     src/ ──colcon build──► build/ + install/ + log/
Your RoboStack Setup: src/ ──pixi build───► .pixi/envs/jazzy/
```

- **Source code**: Lives in `tof_slam_sim/` directory
- **Build output**: Goes directly into `.pixi/envs/jazzy/` (the pixi environment)
- **No separate install directory**: Everything is merged into the environment

## Quick Start: Build Your Project

### Step 1: Open Terminal in Project Root

```bash
cd /Users/hongyilin/projects/gazebo-slam-prototype
```

### Step 2: Build Everything

```bash
pixi run -e jazzy build
```

**What this command does:**
- Downloads and sets up ROS 2 Jazzy if needed
- Compiles your C++/Python code
- Installs everything into the pixi environment
- Makes your package available to ROS commands

### Step 3: Verify It Worked

```bash
# Check if your package is available
pixi run -e jazzy ros2 pkg list | grep tof_slam_sim

# Should output: tof_slam_sim
```

### Step 4: Activate Environment for Development

```bash
pixi shell -e jazzy
```

Now you can run ROS commands directly:
```bash
ros2 launch tof_slam_sim sim_with_bridge.launch.py
```

## Understanding the Build Process

### What Happens During `pixi run -e jazzy build`

1. **Environment Setup**: Pixi activates the ROS 2 Jazzy environment
2. **Dependency Resolution**: Ensures all required packages are installed
3. **Code Compilation**: Compiles C++ and Python code
4. **Package Installation**: Installs your package into `.pixi/envs/jazzy/`
5. **Environment Configuration**: Sets up paths so ROS can find your package

### Build Output Locations

After building, your package files end up in these locations:

```
.pixi/envs/jazzy/
├── lib/python3.12/site-packages/tof_slam_sim/     # Python modules
├── share/tof_slam_sim/                           # Launch files, configs
├── share/ament_index/...                         # ROS package registry
└── bin/                                          # Executables (if any)
```

### Temporary Build Files (Safe to Delete)

During build, these directories are created but can be safely deleted:

```
build/          # Temporary compilation files (normal - always created by colcon)
install/        # Unused in RoboStack (traditional ROS artifact)
log/            # Build logs
```

**Note:** The `build/` directory will always be created by colcon, even in RoboStack. It contains intermediate build artifacts like CMake files, object files, and Makefiles. Your actual package gets installed to `.pixi/envs/jazzy/` while `build/` is just colcon's "workshop". It's safe to delete `build/` anytime - it gets recreated on the next build.

## Common Build Tasks

### Build for the First Time

```bash
pixi run -e jazzy build
```

### Rebuild After Code Changes

```bash
# Quick rebuild (keeps existing build cache)
pixi run -e jazzy build

# OR clean rebuild (starts fresh)
pixi run -e jazzy rebuild
```

### Clean Build Artifacts

```bash
# Remove temporary build files
pixi run -e jazzy clean

# Remove everything and rebuild
pixi run -e jazzy rebuild
```

## Development Workflow

### Making Code Changes

1. **Edit your source code** in `tof_slam_sim/src/`
2. **Rebuild**:
   ```bash
   pixi run -e jazzy build
   ```
3. **Test**:
   ```bash
   pixi run -e jazzy ros2 launch tof_slam_sim sim_with_bridge.launch.py
   ```

### Adding New Files

If you add new source files:

1. **Update** `tof_slam_sim/CMakeLists.txt` (for C++) or package structure
2. **Rebuild**:
   ```bash
   pixi run -e jazzy build
   ```
3. **Verify** new files are installed:
   ```bash
   pixi run -e jazzy ros2 pkg executables tof_slam_sim
   ```

## Package Structure Explained

### Your Package: `tof_slam_sim/`

```
tof_slam_sim/
├── CMakeLists.txt          # Build configuration (C++ compilation)
├── package.xml            # Package metadata (dependencies, etc.)
├── src/                   # Source code
│   ├── tof8x8_to_scan.py   # Converts depth to laser scans
│   ├── scan_merger.py     # Merges multiple laser scans
│   └── test_controller.py # Robot control logic
├── launch/                # Launch configurations
├── models/                # Gazebo model files
├── worlds/                # Gazebo world files
└── config/                # Configuration files
```

### What Each File Does

- **`CMakeLists.txt`**: Tells the build system how to compile C++ code
- **`package.xml`**: Lists what other packages your code needs
- **`src/*.py`**: Your Python ROS nodes (automatically installed as executables)
- **`launch/*.launch.py`**: Startup configurations for running multiple nodes

## Running Your Built Package

### Method 1: Using Pixi Run (Recommended)

```bash
# Launch simulation
pixi run -e jazzy sim

# Launch SLAM
pixi run -e jazzy slam

# Run any ROS command
pixi run -e jazzy ros2 topic list
```

### Method 2: Interactive Shell

```bash
# Enter the ROS environment
pixi shell -e jazzy

# Now you can run ROS commands directly
ros2 launch tof_slam_sim sim_with_bridge.launch.py
ros2 pkg list
exit  # Leave the shell
```

## Troubleshooting Build Issues

### Build Fails with CMake Errors

```bash
# Check build logs
cat log/latest_build/tof_slam_sim/stdout_stderr.log

# Clean and rebuild
pixi run -e jazzy rebuild
```

### Package Not Found After Build

```bash
# Check if environment is activated
pixi run -e jazzy ros2 pkg list | grep tof_slam_sim

# If not found, rebuild
pixi run -e jazzy build
```

### Permission Errors

```bash
# Fix permissions on scripts
chmod +x scripts/*.sh
```

### ROS Commands Not Working

```bash
# Ensure you're using the pixi environment
pixi run -e jazzy ros2 --version

# Check RMW implementation
pixi run -e jazzy env | grep RMW_IMPLEMENTATION
```

## Advanced: Understanding the Pixi Configuration

### Your `pixi.toml` File

```toml
[environments]
jazzy = { features = ["jazzy", "build"] }

[feature.jazzy.dependencies]
ros-jazzy-desktop = "*"  # Full ROS 2 installation
# ... other dependencies

[tasks]
build = { cmd = "colcon build --symlink-install --install-base .pixi/envs/jazzy --merge-install" }
```

This configuration:
- Defines a `jazzy` environment with ROS 2
- Specifies all required dependencies
- Provides build commands that install directly into the pixi environment

### Environment Activation

When you run `pixi shell -e jazzy`:
1. Pixi sets up the isolated environment
2. RoboStack activates ROS 2 automatically
3. Your activation scripts run (configuring CycloneDDS)
4. All ROS tools become available

## Summary: Daily Development Flow

```bash
# Start working
cd /Users/hongyilin/projects/gazebo-slam-prototype

# First time setup
pixi run -e jazzy build

# Development loop
1. Edit code in tof_slam_sim/src/
2. pixi run -e jazzy build
3. pixi run -e jazzy sim  # Test
4. Repeat...

# Clean up when done
pixi run -e jazzy clean
```

## Key Takeaways

1. **No workspace setup needed** - Everything is managed by pixi
2. **Builds go directly into environment** - No separate install directory
3. **Automatic dependency management** - Pixi handles ROS versions
4. **Cross-platform** - Same workflow on macOS, Linux, Windows
5. **CycloneDDS configured automatically** - No manual DDS setup

Your build system is designed to be simple: edit code → build → test. The complexity of ROS package management is handled automatically by RoboStack and Pixi.
