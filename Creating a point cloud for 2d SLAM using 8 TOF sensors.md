#robotics 
*** 
## Software and dependency req
#### 8× VL53L7CX → 2D SLAM — Dependency & Deployment Guide (macOS + Windows)

> Goal: run **everything locally** without PX4 to test whether **eight VL53L7CX-like ToF sensors** (3.5 m range, ~60° FOV, 8×8 zones) are sufficient for **control-in-sim** and **2D SLAM**.  
> Recommended pairing today: **ROS 2 Jazzy ↔ Gazebo Harmonic** with the **ros_gz** bridge.  
> SLAM stack: **slam_toolbox** (Cartographer-like but easier to set up).
---
#### Compatibility Matrix (recommended)

| OS      | ROS 2 | Gazebo              | Bridge         | Notes                                                                               |
| ------- | ----- | ------------------- | -------------- | ----------------------------------------------------------------------------------- |
| macOS   | Jazzy | Harmonic (Homebrew) | ros_gz (Conda) | Solid local workflow incl. GUI.                                                     |
| Windows | Jazzy | Harmonic            | ros_gz         | **Best path** is via **WSL2 (Ubuntu 22.04/24.04)**; native is possible but fussier. |

---
RoboStack is ideal for this stage because:
- ✅ **Cross-platform (macOS, Windows, Linux)**: everything runs inside Conda, so you can prototype directly on your development laptop without spinning up a Linux box.  
- ✅ **Conda-based package management**: avoids system-wide apt conflicts, keeps your ROS workspace isolated, and lets you easily install extras like `numpy`, `opencv`, `pcl`.  
- ✅ **Prebuilt binaries for Apple Silicon (osx-arm64)**: you can run ROS 2, RViz2, and slam_toolbox natively on M1/M2 Macs — no need for Docker emulation or dual boot.  
- ✅ **Quick iteration**: you can drop your own adapter nodes (e.g. ToF 8×8 → LaserScan) into a colcon workspace in the same environment and build immediately.  
- ✅ **Easy reset/rollback**: if your environment breaks, you can just `mamba remove -n ros-jazzy --all` and recreate from a simple `environment.yml`.

For initial prototyping, RoboStack gives you a **fast, lightweight dev loop** while staying compatible with the official ROS ↔ Gazebo ecosystem.  
Later, when you transition to **PX4 SITL + hardware testing**, you’ll likely migrate your simulation stack to a Linux box (Ubuntu + apt) for maximum compatibility — but RoboStack ensures your early code, adapters, and configs are **ROS-native and portable**.

*** 
#### Links:
[Robostack](https://robostack.github.io/GettingStarted.html)

Note a problem with the default install (fastdds) refer to [link](https://github.com/RoboStack/ros-jazzy/issues/57)
(This is automatically configured when u initialize pixi env in /scripts)


[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_osx/)

***
#### Environment Management: 2 Ways (Conda or Pixi)

##### Option 1: Conda (traditional)
# Update the active env from environment.yml
`conda env update -f environment.yml --prune`

##### Option 2: Pixi (modern package manager)
# Save and exit pixi.toml
pixi install -e jazzy
# You can now start an environment with your desired robostack distribution using one of the below commands (either executed from within the project directory or by appending `--manifest-path` and pointing to your project directory):

# ROS jazzy (includes CycloneDDS configuration)
pixi shell -e jazzy


cd robostack
pixi shell -e jazzy

*** 
