# FrontierServer Node

## What It Does

The FrontierServer finds places where robots should explore. It analyzes the merged occupancy grid (`/map`) and detects boundaries between known and unknown space -- these are the frontiers.

## How It Works

### Input: The Map
- Subscribes to `/map` (`nav_msgs/OccupancyGrid`)
- This is the merged map from all robots combined by the `swarm_map_fuser`
- Each cell is: **Free** (0-30), **Occupied** (65-100), or **Unknown** (-1)

### Processing Steps

1. **Get the latest map** from `/map`
2. **Find frontier cells** -- free cells adjacent to unknown cells
3. **Cluster connected frontiers** using flood-fill
4. **Filter small clusters** below `min_frontier_cluster_size`
5. **Compute centroids** of each cluster as exploration targets
6. **Publish** on `/swarm/frontiers`

### Periodic Warning

If no `/map` data has been received, the node logs a `WARN` message periodically to alert that the map fuser may not be running.

## Publications

**Topic**: `/swarm/frontiers`
**Type**: `geometry_msgs/PoseStamped`

Each message contains:
- **Position**: `(x, y, 0.0)` -- frontier centroid in world coordinates
- **Size**: Stored in `orientation.w` -- number of cells in the cluster
- **Frame**: `robot/map`

Example:
```yaml
header:
  frame_id: robot/map
pose:
  position: {x: 5.2, y: 3.8, z: 0.0}
  orientation: {w: 15.0}  # 15 cells in this frontier
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_topic` | `/map` | Which map to analyze |
| `frontiers_topic` | `/swarm/frontiers` | Where to publish goals |
| `free_threshold` | 30 | Max value for "free" cells |
| `occupied_threshold` | 65 | Min value for "occupied" cells |
| `min_frontier_cluster_size` | 6 | Minimum cells for a valid frontier |
| `frontier_ttl_sec` | 10.0 | How long to remember frontiers |
| `update_rate_hz` | 2.0 | How often to scan for frontiers |

## How to Debug

```bash
# See what frontiers are found
ros2 topic echo /swarm/frontiers

# Check publish rate
ros2 topic hz /swarm/frontiers

# Verify map is publishing
ros2 topic hz /map
```

### Common Issues

**"No frontiers published"**
- Check if `/map` topic exists and is publishing
- Check if map has unknown areas (fully explored map has no frontiers)
- Verify map frame is `robot/map`

**"Frontiers disappear quickly"**
- Increase `frontier_ttl_sec`
- Frontiers naturally disappear when explored

**"Too many/few frontiers"**
- Adjust `min_frontier_cluster_size`

## Related Components

- **Map Fuser** (`swarm_map_fuser`): Creates the `/map` input
- **GoalAllocator**: Uses frontiers to assign goals to drones
- **DroneExecutor**: Navigates to frontier locations via Nav2
