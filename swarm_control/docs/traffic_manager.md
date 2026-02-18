# TrafficManager Node

## What It Does

The TrafficManager monitors proximity between all robots and logs warnings when drones get too close. It serves as a safety watchdog for the swarm.

## Current Behavior

### What it does:
- Monitors distance between all robot pairs using TF positions
- Logs `CRITICAL` alerts when distance is below `collision_distance_threshold`
- Logs `WARNING` when distance is below `proximity_log_threshold`
- Skips checks when all drones are at origin (before TF is ready)

### Proximity Monitoring

For every pair of robots, calculates 3D Euclidean distance:

```
distance = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
```

### Warning Levels

| Distance | Level | Action |
|----------|-------|--------|
| < 1.5m | **CRITICAL** | Immediate collision risk logged |
| < 3.0m | **WARNING** | Close proximity logged |
| > 3.0m | **OK** | Safe separation |

## Subscriptions

### `/swarm/drone_states`
- **Type**: `std_msgs/String`
- **Format**: `robot_id:state:x,y,yaw:assignment`
- **Purpose**: Track robot positions for proximity checks

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_states_topic` | `/swarm/drone_states` | Robot status input |
| `collision_distance_threshold` | 1.5 | Critical distance (meters) |
| `proximity_log_threshold` | 3.0 | Warning distance (meters) |
| `update_rate_hz` | 2.0 | Monitoring frequency |

## Initialization Safety

During startup, all drones report position `(0,0,0)` before TF is ready. The TrafficManager detects this condition and skips proximity checks to avoid false `CRITICAL` alerts.

## How to Debug

```bash
# Monitor the node logs for proximity warnings
# (visible in launch terminal output)

# Check drone positions
ros2 topic echo /swarm/drone_states
```

### Common Issues

**"False proximity alerts at startup"**
- Normal if drones haven't received TF data yet
- The node now filters out all-at-origin conditions

**"No proximity monitoring"**
- Check the node is running: `ros2 node list | grep traffic`
- Check drones are publishing states

## Future Capabilities

- Goal modification to avoid collisions
- Path prediction and trajectory conflict detection
- Priority system for emergency robots
- Airspace reservation for maneuvers

## Related Components

- **DroneExecutor**: Publishes position data used for proximity checks
- **GoalAllocator**: Could receive collision avoidance feedback in the future
- **SwarmDashboard**: Proximity events appear in the event log
