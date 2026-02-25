# Troubleshooting Guide

## Common Issues and Solutions

### "No frontiers detected"

**Symptoms:**
- `/swarm/frontiers` topic has no messages
- FrontierServer logs: "No /map received"

**Possible Causes:**
1. **Map not publishing**: The map fuser needs scan data from robots
2. **Map fully explored**: No unknown cells adjacent to free cells
3. **Wrong map topic**: FrontierServer expects `/map`

**Solutions:**
```bash
# Check map is publishing
ros2 topic hz /map

# Check map has data
ros2 topic echo /map --once | head -5

# Verify FrontierServer is running
ros2 node list | grep frontier
```

---

### "No assignments published"

**Symptoms:**
- `/swarm/assignments` topic empty
- GoalAllocator not publishing

**Possible Causes:**
1. **No frontiers available** -- Check FrontierServer first
2. **No robots in AVAILABLE state** -- Drones must complete startup FSM
3. **All drones already assigned** -- Wait for goals to complete

**Solutions:**
```bash
# Check drone states
ros2 topic echo /swarm/drone_states

# Check frontiers exist
ros2 topic hz /swarm/frontiers
```

---

### "Robots stuck in BOOT state"

**Symptoms:**
- DroneExecutor logs: "Waiting for NavigateToPose server"
- Robots not progressing past BOOT

**Possible Causes:**
1. **Nav2 not started** or still initializing (can take 30-60s)
2. **TF transforms not available** yet
3. **Wrong namespace** -- executor namespace doesn't match Nav2 namespace

**Solutions:**
```bash
# Check Nav2 is running
ros2 node list | grep controller_server

# Check TF tree
ros2 run tf2_tools view_frames.py

# Check specific robot TF
ros2 run tf2_ros tf2_echo robot/map robot/base_link
```

**Note:** After 60 seconds without Nav2, DroneExecutors transition to `EMERGENCY`.

---

### "Navigation goals rejected / aborted"

**Symptoms:**
- "Nav2 rejected goal" or "Nav2 aborted" in logs
- Drones entering RECOVERY state

**Possible Causes:**
1. **Invalid goal position** -- frontier outside costmap bounds
2. **Robot in occupied space** -- costmap shows robot's start as blocked
3. **Planner failure** -- no valid path to goal

**Solutions:**
```bash
# Check TF transforms
ros2 run tf2_ros tf2_echo robot/map robot/odom

# Check Nav2 costmap
ros2 topic echo /local_costmap/costmap --once | head -10
```

---

### "Mission stuck in IDLE"

**Symptoms:**
- MissionSupervisor not progressing
- "Waiting for drones" in logs

**Possible Causes:**
1. **Robots not publishing states** -- DroneExecutors may not be running
2. **Wrong expected_drones parameter** -- parameter doesn't match actual robots
3. **Drones haven't reached ready states** -- still in BOOT

**Solutions:**
```bash
# Check expected drones
ros2 param get /mission_supervisor expected_drones

# Check drone state publications
ros2 topic echo /swarm/drone_states
```

**Note:** MissionSupervisor auto-starts after 15 seconds even with missing drones.

---

### "Dashboard not loading"

**Symptoms:**
- `http://localhost:8080` returns connection refused
- No dashboard page in browser

**Possible Causes:**
1. **Dashboard node not running** -- check launch argument `dashboard:=true`
2. **Port 8080 in use** -- another process using the port
3. **Node crashed** -- check logs

**Solutions:**
```bash
# Check dashboard node is running
ros2 node list | grep dashboard

# Check if port 8080 is in use
lsof -i :8080

# Check node logs for errors
# (visible in launch terminal output)
```

---

### "TF_OLD_DATA warnings"

**Symptoms:**
- Repeated "TF_OLD_DATA" warnings in DroneExecutor logs

**Possible Causes:**
- Normal during startup before all transforms are publishing
- TF buffer cache may retain stale data

**Solutions:**
- These warnings are expected during the first 10-20 seconds
- DroneExecutors use a 5-second TF cache to minimize this
- If persistent, check the TF broadcaster node is running

---

### "High CPU usage"

**Symptoms:**
- System running slow
- Gazebo framerate dropping

**Possible Causes:**
1. **Too many Nav2 stacks** -- 5 concurrent Nav2 instances is resource-intensive
2. **Sensor processing** -- 40 depth cameras (5 robots x 8 sensors each)

**Solutions:**
- Reduce `num_robots` if testing on limited hardware
- Check Gazebo physics step size (currently 4ms for performance)

---

## Diagnostic Commands

### System Health Check

```bash
# Check all nodes are running
ros2 node list

# Check swarm topics
ros2 topic list | grep swarm

# Check TF tree
ros2 run tf2_tools view_frames.py
```

### Topic Monitoring

```bash
# Monitor publish rates
ros2 topic hz /swarm/frontiers
ros2 topic hz /swarm/assignments
ros2 topic hz /swarm/drone_states
ros2 topic hz /swarm/mission_state
ros2 topic hz /map
```

### Map Inspection

```bash
# Check map is publishing
ros2 topic echo /map --once | head -20

# Visualize map in RViz
ros2 run rviz2 rviz2 -d config/slam.rviz
```

---

## Recovery Procedures

### Soft Reset
```bash
# Ctrl+C the launch file, then restart
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py
```

### Hard Reset
```bash
# Kill all related processes
pkill -f ros2
pkill -f gz
pkill -f ruby  # Gazebo GUI process

# Restart from scratch
pixi run -e jazzy ros2 launch swarm_control test_swarm.launch.py
```

### Emergency Stop
```bash
# Send zero velocity to all robots
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"
ros2 topic pub --once /robot2/cmd_vel geometry_msgs/Twist "{}"
ros2 topic pub --once /robot3/cmd_vel geometry_msgs/Twist "{}"
ros2 topic pub --once /robot4/cmd_vel geometry_msgs/Twist "{}"
ros2 topic pub --once /robot5/cmd_vel geometry_msgs/Twist "{}"
```

---

## Getting Help

When asking for help, include:

1. **Launch command** you used
2. **Error logs** from the failing node
3. **Dashboard screenshot** if available
4. **Topic outputs** showing the issue
5. **System info**: OS, `pixi --version`
