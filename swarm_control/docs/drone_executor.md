# DroneExecutor Node

## What It Does

The DroneExecutor is a per-robot FSM that controls one specific robot's behavior. Each robot gets its own executor that handles everything from boot to goal execution, using Nav2's `NavigateToPose` action for navigation.

## State Machine

Each robot follows a strict sequence of states:

```
BOOT -> PREFLIGHT -> ARMED -> TAKING_OFF -> STAGING -> AVAILABLE -> EXECUTING_GOAL -> ...
```

### States

| State | What Happens | Transitions To |
|-------|-------------|---------------|
| **BOOT** | Wait for Nav2 `NavigateToPose` server (60s timeout) | PREFLIGHT or EMERGENCY |
| **PREFLIGHT** | TF readiness check | ARMED |
| **ARMED** | Ready for takeoff, waiting for mission | TAKING_OFF |
| **TAKING_OFF** | Skipped in simulation (direct transition) | STAGING |
| **STAGING** | At cruise position, waiting for mission start | AVAILABLE |
| **AVAILABLE** | Ready for frontier assignments | EXECUTING_GOAL |
| **EXECUTING_GOAL** | Navigating to frontier via Nav2 | AVAILABLE or RECOVERY |
| **RECOVERY** | Handle navigation failures | EXECUTING_GOAL or EMERGENCY |
| **RETURNING** | Go home (mission ending) | LANDING |
| **LANDING** | Descend to ground | LANDED |
| **LANDED** | On ground, mission finished | (terminal) |
| **EMERGENCY** | Safety stop -- Nav2 timeout or critical failure | (terminal) |

## Nav2 Integration

### How Navigation Works

1. **Receive Assignment**: Parses `robot_id:frontier_id:cx:cy` from `/swarm/assignments`
2. **Construct Goal**: Creates a `PoseStamped` in `robot/map` frame at `(cx, cy, 0.0)` with yaw toward the goal
3. **Send Goal**: Calls `NavigateToPose` action asynchronously
4. **Monitor**: Watches for success, failure, or `goal_timeout_sec`
5. **On Success**: Transitions to `AVAILABLE` for next assignment
6. **On Failure**: Transitions to `RECOVERY` with retry logic

### Yaw Calculation

The executor computes yaw from the drone's current position toward the goal:
```python
yaw = atan2(goal_y - current_y, goal_x - current_x)
```

### Shutdown Cleanup

When the node is destroyed (e.g., Ctrl+C), any active Nav2 goal is cancelled:
```python
def destroy_node(self):
    self._cancel_nav_goal()
    super().destroy_node()
```

## Subscriptions

### `/swarm/assignments`
- **Type**: `std_msgs/String`
- **Format**: `robot_id:frontier_id:cx:cy`
- **Action**: If `robot_id` matches this executor's robot, start executing the goal

## Publications

### `/swarm/drone_states`
- **Type**: `std_msgs/String`
- **Format**: `robot_id:state:x,y,yaw:assignment`
- **Content**: Current FSM state, position from TF, and active assignment

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_namespace` | `""` | Robot namespace (empty for `robot`, `robot2` for second, etc.) |
| `assignments_topic` | `/swarm/assignments` | Assignment input |
| `drone_states_topic` | `/swarm/drone_states` | Status output |
| `goal_timeout_sec` | 45.0 | Navigation timeout before recovery |
| `recovery_timeout_sec` | 15.0 | Recovery attempt time |
| `cruise_altitude` | 1.2 | Flight altitude (meters) |

## Behavior Examples

### Normal Exploration Flow
```
BOOT -> PREFLIGHT -> ARMED -> TAKING_OFF -> STAGING -> AVAILABLE
                                                         |
                                              Assignment received
                                                         |
EXECUTING_GOAL -> Goal reached -> AVAILABLE -> Next assignment...
```

### Handling Navigation Failures
```
EXECUTING_GOAL -> Nav2 aborts -> RECOVERY -> Retry -> EXECUTING_GOAL
EXECUTING_GOAL -> Timeout -> RECOVERY -> Give up -> AVAILABLE
```

### Emergency (Nav2 not available)
```
BOOT -> (60 seconds, no Nav2 server) -> EMERGENCY
```

## How to Debug

```bash
# Check robot state
ros2 topic echo /swarm/drone_states | grep robot

# Check Nav2 action status
ros2 action list

# Check TF for specific robot
ros2 run tf2_ros tf2_echo robot/map robot/base_footprint
```

### Common Issues

**"Stuck in BOOT"**
- Nav2 not running or still initializing
- Check `ros2 node list | grep controller_server`

**"Goals rejected"**
- Frontier position may be in occupied space on the costmap
- Check map frame alignment

**"Constant RECOVERY"**
- Nav2 planner can't find paths -- check costmap configuration
- Goal may be unreachable (inside a wall)

## Related Components

- **GoalAllocator**: Publishes assignments with frontier coordinates
- **Nav2 Stack**: Handles actual path planning and control
- **MissionSupervisor**: Coordinates takeoff timing and mission phases
