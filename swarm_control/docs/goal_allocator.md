# GoalAllocator Node

## What It Does

The GoalAllocator is a smart dispatcher that decides which robot should explore which frontier. It solves the problem: "We have drones available and frontiers to explore -- who should go where?"

## How It Decides

### Scoring Formula

For each robot-frontier pair, it calculates:

```
Score = (gain_weight x frontier_size) - (cost_weight x distance)
```

- **Frontier Size** = How many cells in the frontier cluster (bigger = more unknown area)
- **Distance** = Euclidean distance from drone to frontier centroid
- **Weights** balance exploration value vs travel time

### Example Calculation

```
Robot A at (0,0):
- Frontier 1: 20 cells, 5m away  -> Score = (1.0 x 20) - (0.35 x 5) = 18.25
- Frontier 2: 10 cells, 2m away  -> Score = (1.0 x 10) - (0.35 x 2) = 9.3

Robot A should explore Frontier 1 (higher score)
```

### Greedy Algorithm

1. Find the best robot-frontier pair (highest score)
2. Assign that robot to that frontier
3. Remove both from consideration
4. Repeat until no more assignments possible

## Subscriptions

### `/swarm/frontiers`
- **Type**: `geometry_msgs/PoseStamped`
- **Content**: Available exploration goals (centroid position, cluster size in `orientation.w`)

### `/swarm/drone_states`
- **Type**: `std_msgs/String`
- **Content**: Robot status -- position, FSM state, current assignment
- **Format**: `robot_id:state:x,y,yaw:assignment`

## Publications

### `/swarm/assignments`
- **Type**: `std_msgs/String`
- **Format**: `robot_id:frontier_id:cx:cy`
- **Example**: `robot:f_042:5.200:3.800`

The format includes the frontier centroid coordinates so DroneExecutors can navigate directly without a separate lookup.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `frontiers_topic` | `/swarm/frontiers` | Frontier input topic |
| `assignments_topic` | `/swarm/assignments` | Assignment output topic |
| `drone_states_topic` | `/swarm/drone_states` | Robot status input |
| `gain_weight` | 1.0 | How much to value frontier size |
| `cost_weight` | 0.35 | How much to penalize distance |
| `assignment_timeout_sec` | 30.0 | How long assignments last before re-evaluation |
| `update_rate_hz` | 2.0 | Decision frequency |

## Anti-Thrashing

Assignments don't change immediately when new frontiers appear. The `assignment_timeout_sec` prevents robots from constantly switching goals.

## How to Debug

```bash
# See assignments
ros2 topic echo /swarm/assignments

# Check available drones
ros2 topic echo /swarm/drone_states

# Check frontier availability
ros2 topic hz /swarm/frontiers
```

### Common Issues

**"No assignments published"**
- Are robots in `AVAILABLE` state? Check drone states.
- Are there frontiers available? Check FrontierServer.
- All drones may already be assigned and executing goals.

**"Poor assignments"**
- Adjust `gain_weight` vs `cost_weight`
- Check distance calculations -- verify TF transforms are correct

**"Assignments change too often"**
- Increase `assignment_timeout_sec`

## Related Components

- **FrontierServer**: Provides exploration goals
- **DroneExecutor**: Receives assignments and executes them via Nav2
- **MissionSupervisor**: Can influence assignment priorities through mission state
