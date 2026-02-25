# MissionSupervisor Node

## What It Does

The MissionSupervisor is the mission commander that oversees the entire swarm. It coordinates takeoff windows, tracks mission phases, and handles emergencies.

## Mission Phases

```
IDLE -> READY -> TAKEOFF_WINDOW_A -> TAKEOFF_WINDOW_B -> RUNNING -> RETURN_AND_LAND -> COMPLETE
```

### Phase Details

### IDLE: Waiting for Setup
- Waits for all expected drones to report ready states (`ARMED`, `STAGING`, `AVAILABLE`, etc.)
- **Auto-start**: After 15 seconds, proceeds even if some drones are missing

### READY: All Systems Go
- All drones detected and ready
- Immediately transitions to takeoff

### TAKEOFF_WINDOW_A: First Group Takes Off
- First group of drones (robot, robot2) can take off
- 30-second window (configurable)
- Advances early if all group drones are staged

### TAKEOFF_WINDOW_B: Second Group Takes Off
- Second group of drones (robot3, robot4, robot5) can take off
- 30-second window
- Advances early if all group drones are staged

### RUNNING: Exploration Phase
- Normal exploration operations
- Monitors for emergency conditions (any drone in `EMERGENCY` state -> `ABORT`)

### RETURN_AND_LAND: Mission End
- Commands all robots to return home and land
- Completes when all robots report `LANDED`

### COMPLETE: Success
- Mission finished successfully

### ABORT: Emergency
- Triggered by any drone entering `EMERGENCY` state
- Mission halted

## Subscriptions

### `/swarm/drone_states`
- **Type**: `std_msgs/String`
- **Format**: `robot_id:state:x,y,yaw:assignment`
- **Tracks**: Each robot's FSM state to determine mission readiness

## Publications

### `/swarm/mission_state`
- **Type**: `std_msgs/String`
- **Values**: `IDLE`, `READY`, `TAKEOFF_WINDOW_A`, `TAKEOFF_WINDOW_B`, `RUNNING`, `RETURN_AND_LAND`, `COMPLETE`, `ABORT`
- **Published**: Every 1 second (FSM tick rate)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mission_state_topic` | `/swarm/mission_state` | State output topic |
| `drone_states_topic` | `/swarm/drone_states` | Robot status input |
| `expected_drones` | `['robot', 'robot2', 'robot3', 'robot4', 'robot5']` | Which robots to expect |
| `takeoff_window_duration_sec` | 30.0 | How long each takeoff window lasts |
| `auto_start_mission` | true | Start automatically when ready |

## Takeoff Group Logic

Drones are split into two groups at startup:
- **Group A**: First half of `expected_drones` (e.g., robot, robot2)
- **Group B**: Second half (e.g., robot3, robot4, robot5)

Each group gets a 30-second takeoff window. The window closes early if all drones in the group reach staged/available state.

## Emergency Handling

If any drone enters `EMERGENCY` state during `RUNNING`, the supervisor transitions to `ABORT`. This provides a safety mechanism for critical failures.

## How to Debug

```bash
# Check mission state
ros2 topic echo /swarm/mission_state

# Monitor drone states
ros2 topic echo /swarm/drone_states

# Check expected drones parameter
ros2 param get /mission_supervisor expected_drones
```

### Common Issues

**"Stuck in IDLE"**
- Drones not publishing states -- check DroneExecutors are running
- `expected_drones` parameter doesn't match actual robots
- Wait for auto-start (15 seconds)

**"Takeoff windows too short"**
- Increase `takeoff_window_duration_sec`

## Related Components

- **DroneExecutor**: Reports state, receives takeoff timing indirectly via mission state
- **GoalAllocator**: Assignment logic runs during `RUNNING` phase
- **SwarmDashboard**: Displays mission state in the header bar
