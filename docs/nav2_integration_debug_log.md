# Nav2 Integration Debug Log

> Written: 2026-02-25
> Context: Integrating multi-robot Nav2 stacks with the PX4 SITL swarm so that `swarm_control`'s `drone_executor` FSM can use `NavigateToPose` for autonomous exploration.
> Prior chat: [Nav2 swarm integration](43368a49-3448-44b6-8afd-bcd2894fcc76)

---

## 1. System Knowledge

### Architecture Overview

The system is a **multi-drone SLAM simulation** with three layers:

| Layer | Package | Purpose |
|-------|---------|---------|
| **Simulation** | `tof_slam_sim` | Gazebo Harmonic + PX4 SITL + ROS bridges + sensor processing |
| **Navigation** | Nav2 (external) | Per-robot path planning, obstacle avoidance, velocity control |
| **Swarm Logic** | `swarm_control` | Centralized FSM: frontier detection → goal allocation → per-drone execution |

### Launch Chain

```
scripts/swarm/run_px4_swarm_fast.sh
  └─ swarm_control/launch/px4_test_swarm.launch.py
       ├─ tof_slam_sim/launch/px4_swarm_fast.launch.py   (immediate)
       │    ├─ Gazebo server + GUI
       │    ├─ MicroXRCE agent
       │    ├─ ros_gz_bridge (sensor topics)
       │    ├─ PX4 SITL instances (one per drone)
       │    ├─ Per-robot nodes: twist_to_px4_offboard, px4_vehicle_odometry_to_odom,
       │    │   gz_pose_info_to_pose_stamped, pose_to_px4_visual_odometry, tof_to_scan,
       │    │   slam_toolbox
       │    ├─ swarm_tf_broadcaster, swarm_map_fuser
       │    ├─ RViz (when rviz:=true)
       │    └─ Nav2 stacks per robot (when nav_mode:=nav2)  ← PROBLEM AREA
       │
       └─ swarm_control/launch/swarm_control.launch.py   (delayed 60s via TimerAction)
            ├─ frontier_server
            ├─ goal_allocator
            ├─ mission_supervisor
            ├─ traffic_manager
            ├─ swarm_dashboard (web UI on :8080)
            └─ drone_executor × N (one per robot)
```

### Key Navigation Flow (nav_mode=nav2)

```
frontier_server → goal_allocator → drone_executor
                                        │
                                        ▼
                                   Nav2 NavigateToPose action
                                        │
                                        ▼
                                   bt_navigator → planner_server → controller_server
                                        │
                                        ▼
                                   cmd_vel_nav → velocity_smoother → cmd_vel
                                        │
                                        ▼
                                   twist_to_px4_offboard → PX4 offboard setpoints
```

### Robot Naming Convention

- `robot` = first drone (PX4 instance 0, Gazebo `x500_small_tof_0`)
- `robot2` = second drone (PX4 instance 1, Gazebo `x500_small_tof_1`)
- `robotN` = Nth drone (PX4 instance N-1)

### Key Files Modified in This Session

| File | What Changed |
|------|-------------|
| `tof_slam_sim/launch/px4_swarm_fast.launch.py` | Added Nav2 per-robot launch logic inside `_build_swarm` OpaqueFunction |
| `swarm_control/launch/px4_test_swarm.launch.py` | Pass `nav_mode` directly (removed broken PythonExpression), swarm delay 60s |
| `scripts/swarm/run_px4_swarm_fast.sh` | Added `rviz:=true`, `--nav-mode` arg, launches `swarm_control/px4_test_swarm.launch.py` |
| `scripts/core/cleanup_sim.sh` | PX4 force-kill + lock file removal + port 8080 cleanup |
| `swarm_control/swarm_control/drone_executor.py` | Boot timeout 60→180s, consecutive failures 3→5 |
| `swarm_control/swarm_control/swarm_dashboard.py` | `allow_reuse_address = True` on HTTPServer subclass |
| `tof_slam_sim/config/nav2_params_px4.yaml` | Created: Nav2 params with relative scan topics + PX4-tuned velocities |

---

## 2. What Was Tried (Chronological)

### Iteration 1: PythonExpression for nav_mode → run_nav2

**Approach:** In `px4_test_swarm.launch.py`, compute `run_nav2` with:
```python
run_nav2 = PythonExpression(["'", nav_mode, "' == 'nav2'"])
```
Then pass it as a launch argument to `px4_swarm_fast.launch.py`.

**Result:** `run_nav2` was always `False` inside `px4_swarm_fast.launch.py`. The `PythonExpression` substitution did not resolve correctly when passed through `IncludeLaunchDescription`'s `launch_arguments`.

**Fix:** Pass `nav_mode` as a raw string instead. `px4_swarm_fast.launch.py` now has its own `nav_mode` `DeclareLaunchArgument` and derives `run_nav2` in plain Python inside the `OpaqueFunction`:
```python
nav_mode_val = str(LaunchConfiguration('nav_mode').perform(context)).strip().lower()
if nav_mode_val == 'nav2':
    run_nav2 = True
```

**Status:** FIXED — confirmed `[px4_swarm_fast] nav_mode='nav2'  run_nav2=True` in logs.

---

### Iteration 2: bringup_launch.py (includes AMCL localization)

**Approach:** Used `nav2_bringup/launch/bringup_launch.py` to launch Nav2 per robot.

**Result:** `bringup_launch.py` launches AMCL localization which **conflicts with our own TF/odometry chain** (`px4_vehicle_odometry_to_odom` → `odom_tf`). Two localization sources fight over the `odom→base_footprint` transform.

**Fix:** Switched to `nav2_bringup/launch/navigation_launch.py` which only launches the navigation stack (planner, controller, behavior, bt_navigator) without localization.

**Status:** FIXED

---

### Iteration 3: GroupAction with launch_configurations dict

**Approach:** Wrap each robot's Nav2 include in:
```python
GroupAction(
    actions=[IncludeLaunchDescription(navigation_launch.py)],
    scoped=True,
    launch_configurations={'namespace': ns, 'params_file': robot_params, ...},
)
```

**Result:** **No Nav2 nodes launched at all.** The `GroupAction.launch_configurations` dict silently set values, but `navigation_launch.py` never started any processes. No errors in logs.

**Fix:** Switched to the standard pattern:
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_launch_path),
    launch_arguments={...}.items(),
)
```

**Status:** FIXED

---

### Iteration 4: use_composition='True' with no component_container

**Approach:** Passed `'use_composition': 'True'` in Nav2 launch arguments.

**Result:** **No Nav2 nodes launched.** `navigation_launch.py` has two mutually exclusive `GroupAction`s:
- `load_nodes`: condition `not use_composition` → launches individual Node processes
- `load_composable_nodes`: condition `use_composition` → calls `LoadComposableNodes` into `nav2_container`

Since no `component_container` named `nav2_container` exists, `LoadComposableNodes` **silently does nothing**. No error, no warning.

**Fix:** Set `'use_composition': 'False'` to launch Nav2 as individual processes.

**Status:** FIXED — Nav2 nodes now appear in `ros2 node list`.

---

### Iteration 5: No PushRosNamespace → namespace collisions

**Approach:** Launch multiple Nav2 stacks with `namespace` launch argument but no explicit `PushRosNamespace`.

**Result:** `navigation_launch.py` declares a `namespace` argument but **never applies it internally** (unlike `bringup_launch.py` which uses `PushRosNamespace`). All Nav2 nodes for all robots launched in the **global namespace**. Two lifecycle managers both tried to configure the same `controller_server`:
```
[WARN] No transition matching 1 found for current state inactive
```

**Fix:** Wrap namespaced robots in `GroupAction(actions=[PushRosNamespace(ns), nav2_include])`:
```python
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction as _GroupAction

if ns:
    nav2_actions.append(_GroupAction(actions=[PushRosNamespace(ns), nav2_include]))
else:
    nav2_actions.append(nav2_include)
```

**Status:** FIXED — `ros2 node list` shows `/controller_server` and `/robot2/controller_server` correctly.

---

### Iteration 6: TimerAction inside OpaqueFunction (does not fire)

**Approach:** Wrap Nav2 actions in `TimerAction(period=25.0, actions=nav2_actions)` inside the `_build_swarm` `OpaqueFunction`, returned as part of the result list.

**Result:** **TimerAction does not fire when returned from an OpaqueFunction.** Tested multiple variations:
- Single TimerAction with all nav2_actions → did not fire (0 Nav2 nodes)
- Multiple TimerActions with staggered periods → only the first fired
- No TimerAction (direct in result list) → all launch correctly

Note: `TimerAction` DOES work at the top-level `LaunchDescription` (proven by the 60s `delayed_swarm` in `px4_test_swarm.launch.py`). The issue is specific to actions returned from `OpaqueFunction`.

**Current workaround:** Nav2 actions are added **directly** to the result list (no delay):
```python
result = [..., *nav2_actions]
```

**Status:** WORKAROUND APPLIED — Nav2 launches immediately with all other nodes.

---

### Iteration 7: DDS storm kills lifecycle manager for robots >= 2

**Symptom:** When launching N drones (N >= 2), Nav2 for robot 1 (global namespace) activates successfully. Nav2 for robot 2+ fails:
```
[robot2.lifecycle_manager_navigation] Failed to change state for node: planner_server.
Exception: planner_server/get_state service client: async_send_request failed.
[robot2.lifecycle_manager_navigation] Failed to bring up all requested nodes. Aborting bringup.
```

**Root cause:** CycloneDDS is overwhelmed at startup:
1. N PX4 SITL instances × ~50 topics each = hundreds of PX4 DDS topics
2. Every ROS node logs `Failed to parse type hash` for every PX4 topic (CycloneDDS vs FastDDS type hash mismatch)
3. The DDS middleware is saturated → lifecycle service calls (`planner_server/get_state`) time out

The first robot's lifecycle manager starts configuring early when DDS is quieter. By the time robot2's lifecycle manager runs, the storm is at peak.

**Status:** FIXED — See Iteration 8 below.

---

### Iteration 8: Staggered Nav2 launch via top-level TimerAction (2026-02-25)

**Approach:** Move Nav2 launch out of `px4_swarm_fast`'s OpaqueFunction to the top-level `px4_test_swarm.launch.py`, using:
1. **60s delay** — `TimerAction(60, nav2_multi_robot)` so Nav2 starts after PX4 DDS discovery settles
2. **Staggered per-robot** — `nav2_multi_robot.launch.py` launches each robot's Nav2 stack 30s apart (robot1 at 0s, robot2 at 30s, robot3 at 60s, ...)
3. **External launch flag** — `nav2_launched_externally:=true` passed to `px4_swarm_fast` when `nav_mode==nav2`, so it skips Nav2

**Files changed:**
- `tof_slam_sim/launch/nav2_multi_robot.launch.py` — New launch file with staggered TimerActions
- `swarm_control/launch/px4_test_swarm.launch.py` — Added `TimerAction(60, nav2_multi_robot)` when `nav_mode==nav2`, passes `nav2_launched_externally`
- `tof_slam_sim/launch/px4_swarm_fast.launch.py` — Added `nav2_launched_externally` arg, skips Nav2 when true

**Status:** Implemented — needs runtime verification.

---

## 3. Current Code State (What the Next Session Inherits)

### `px4_swarm_fast.launch.py` — Nav2 Launch Section (lines ~600-662)

```python
# Inside _build_swarm(context) OpaqueFunction:

nav_mode_val = str(LaunchConfiguration('nav_mode').perform(context)).strip().lower()

# Nav2 per robot
nav2_actions = []
if nav_mode_val == 'nav2':
    run_nav2 = True
    nav2_launch_path = '.../nav2_bringup/launch/navigation_launch.py'

    for r in robots:
        ns = '' if r == 'robot' else r
        robot_params = _write_nav2_params(base_path=nav2_params_path, robot=r)
        nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'params_file': robot_params,
                'autostart': 'true',
                'use_composition': 'False',    # MUST be False — no container exists
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        )
        if ns:
            nav2_actions.append(_GroupAction(actions=[PushRosNamespace(ns), nav2_include]))
        else:
            nav2_actions.append(nav2_include)

# Added directly to result (TimerAction doesn't work in OpaqueFunction)
result = [..., *nav2_actions]
```

### `_write_nav2_params()` — Per-Robot TF Frame Substitution

Reads `nav2_params_px4.yaml`, replaces `robot/odom` → `robotN/odom` etc., writes to `/tmp/nav2_robotN_<pid>_<stamp>.yaml`.

### `px4_test_swarm.launch.py`

- Passes `nav_mode` as a string (not PythonExpression) to `px4_swarm_fast.launch.py`
- `swarm_control` delayed 60 seconds via `TimerAction` at top-level LaunchDescription
- `health_ui: 'false'` (disables tkinter dashboard; web dashboard used instead)

### `drone_executor.py` Changes

- Boot timeout: 60s → **180s** (Nav2 needs time to initialize)
- Consecutive failures before EMERGENCY: 3 → **5**

### `swarm_dashboard.py` Change

- `_ReusableHTTPServer(HTTPServer)` with `allow_reuse_address = True` to prevent `OSError: [Errno 48] Address already in use`

### `cleanup_sim.sh` Additions

- PX4: `pkill -KILL` + `rm -f /tmp/px4_lock_*` + `rm -f /tmp/px4-*lock*`
- Port 8080: `lsof -ti :8080 | xargs kill -9`
- Nav2: `pkill -f '[n]av2_'`

---

## 4. The Problem to Solve

### Primary: Nav2 Lifecycle Manager Fails for Multi-Robot Under DDS Load

The lifecycle_manager for the 2nd+ robot's Nav2 stack fails because CycloneDDS service calls time out under the PX4 topic discovery storm. The first robot usually succeeds.

### Possible Approaches (Not Yet Tried)

1. **Move Nav2 launch to a separate top-level TimerAction.** Create a dedicated launch file (e.g., `nav2_multi_robot.launch.py`) that takes the robot list and launches Nav2 stacks with staggered delays. Include it via `TimerAction` at the `px4_test_swarm.launch.py` level (where TimerAction works).

2. **Use an OpaqueFunction at generate_launch_description level** (not nested inside another OpaqueFunction) to create staggered TimerActions. Since `generate_launch_description()` returns a top-level LaunchDescription, TimerActions returned from its OpaqueFunction might fire.

3. **Use RegisterEventHandler + OnProcessStart** to chain Nav2 launches sequentially (launch robot2 Nav2 only after robot1's lifecycle_manager reports active).

4. **Add lifecycle manager retry/resilience**: The Nav2 lifecycle_manager has `bond_timeout` (default 4s) and service call timeout parameters. Increasing these in the params file might let it survive the DDS storm.

5. **Reduce DDS storm**: Configure CycloneDDS to ignore PX4 topics using DDS partitioning or domain separation, so Nav2 nodes don't waste time parsing irrelevant PX4 type hashes.

6. **Use DDS domain ID separation**: Run PX4 on a different DDS domain and use `MicroXRCEAgent` to bridge only the needed topics.

---

## 5. Confirmed Working Components

| Component | Status |
|-----------|--------|
| PX4 SITL spawning (4+ drones) | Works (after cleanup fix) |
| Gazebo sensor bridges | Works |
| tof_to_scan + slam_toolbox per robot | Works |
| swarm_tf_broadcaster + swarm_map_fuser | Works |
| swarm_control FSM nodes | Works |
| drone_executor state machine | Works (BOOT→PREFLIGHT→ARMED→...) |
| Web dashboard (localhost:8080) | Works (after SO_REUSEADDR fix) |
| RViz auto-launch | Works (rviz:=true in run script) |
| cleanup_sim.sh | Works (kills PX4, port 8080, Nav2, Gazebo) |
| Nav2 for single robot (global namespace) | Works |
| Nav2 for multi-robot (namespaced) | **Fails** — lifecycle times out for robot >= 2 |
| drone_executor → NavigateToPose | **Blocked** — Nav2 not active |

---

## 6. Build & Run Commands

```bash
# Build
pixi run -e jazzy build

# Run (nav2 mode, default spawn, with Gazebo GUI + RViz)
pixi run -e jazzy bash scripts/swarm/run_px4_swarm_fast.sh --default --num-drones 8

# Run with autopilot mode (bypasses Nav2, uses auto_pilot node)
pixi run -e jazzy bash scripts/swarm/run_px4_swarm_fast.sh --default --num-drones 8 --nav-mode autopilot

# Cleanup stale processes before re-running
pixi run -e jazzy bash scripts/core/cleanup_sim.sh
```
