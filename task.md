---

# Swarm Refactor Design Doc (High-Level)

## Goals of the rework

1. **Eliminate duplicated compute**: stop running frontier detection / allocation logic per drone.
2. **Centralize exploration intelligence**: compute frontiers once on the merged map; assign drones centrally.
3. **Keep low-level control unchanged**: no changes to PX4 bridge / Nav2 internals for now.
4. **Add structure + observability**: explicit FSMs and message contracts so behavior is debuggable.
5. **Leave room for future**: traffic manager exists but is a passthrough; can be filled later.

---

# Architecture Overview

## Before (current)

* Each drone:

  * has a mission manager / explorer
  * scans map for frontiers
  * does its own voronoi / lane restriction
  * sends goals into its local Nav2 stack

**Downside:** repeated map scans and inconsistent frontier selection at scale.

## After (proposed)

### Central nodes (new)

* **`frontier_server`**
* **`goal_allocator`**
* **`mission_supervisor`** (global mission FSM + takeoff window orchestration)
* **`traffic_manager`** *(stub passthrough)*

### Per-drone node (new or refactor)

* **`drone_executor`** (per drone FSM + “goal consumer”)

### Existing nodes (unchanged)

* Map merging pipeline
* Nav2 per drone (or whatever your codebase does)
* PX4 bridge/control stack

---

# Required New Concepts

## 1) Shared “Swarm World Model”

Coding agent should implement a minimal internal model containing:

* `MergedMap` (occupancy grid + metadata)
* `DroneRegistry` (drone_id → pose + health + FSM state)
* `FrontierSet` (frontier_id → centroid + size + status)
* `Assignments` (drone_id → frontier_id + timestamp)

This doesn’t require a new database; in ROS this can just be state in nodes.

---

## 2) Frontier Server (central)

### Responsibility

Compute frontier targets *once* from the **merged map**, publish them for allocation.

### Inputs (discover in codebase)

* merged occupancy grid topic
* map metadata (if separate)
* frame info (`map`)

### Output (new)

* `/swarm/frontiers` (message type can be custom or JSON in std_msgs for v0)

### Algorithm (v0)

Use your existing neighbor-check frontier definition:

* a **frontier cell** = free cell adjacent (4-neigh) to unknown
* cluster frontier cells into connected components
* compute centroid and size
* publish list of frontier targets

### Stability requirement (important)

Frontiers should not flicker wildly between ticks. Add simple smoothing:

* frontier ID = hash of centroid grid cell (or nearest cluster label)
* maintain a frontier TTL
* only delete after missing K updates

> The point is to reduce thrashing in allocation.

---

## 3) Goal Allocator (central)

### Responsibility

Given drone poses + frontier targets, decide “best drone → best frontier”.

### Inputs (discover)

* a swarm drone pose stream already exists or must be aggregated
* `/swarm/frontiers`

### Output (new)

* `/swarm/assignments`

### Allocation policy (v0)

Simple and effective:

* Only consider drones whose FSM state is `AVAILABLE` (idle/exploring-ready)
* Only consider frontiers not already claimed or recently assigned
* Score = `gain - λ * distance`

  * `gain` = frontier.size
  * `distance` = Euclidean (for now; later swap to path cost)
* Greedy selection:

  * choose highest scoring pair, assign, remove drone + frontier from pool, repeat

### Anti-thrashing rules (must)

* A drone keeps its assignment until:

  * reached goal (success)
  * goal invalid (planner rejects / frontier disappears)
  * timeout
* Allocator should not reassign drones every cycle unless necessary.

---

## 4) Drone Executor FSM (per drone)

### Responsibility

Consume assignments and execute them via existing navigation/control pipeline.

### Key constraint

**Do not alter bridge/Nav2 internals**. The executor must adapt to what’s already there.

### Drone FSM (competition-aligned)

Each drone runs an FSM that matches “operational phases”, not exploration heuristics.

**States**

* `BOOT`
* `PREFLIGHT`
* `ARMED`
* `TAKING_OFF`
* `STAGING`
* `AVAILABLE`  ← important for allocator
* `EXECUTING_GOAL`
* `RECOVERY`
* `RETURNING`
* `LANDING`
* `LANDED`
* `EMERGENCY`

**Events**

* ready checks passed
* mission supervisor state changes (RUNNING / RETURN_AND_LAND)
* assignment received
* navigation success/failure
* timeout/stuck
* emergency triggers

### Behavior (v0)

* In `AVAILABLE`:

  * wait for assignment
  * once assigned → send goal to navigation stack
* In `EXECUTING_GOAL`:

  * monitor completion and report status
* In `RECOVERY`:

  * attempt replanning / small retreat / rotate in place (whatever existing stack supports)
  * if fail, mark drone unavailable and notify supervisor

**Status output**

* `/swarm/drone_states`: (id, pose, fsm_state, current_assignment, health flags)

---

## 5) Mission Supervisor (global FSM)

### Responsibility

Coordinate *global run phases* and enforce “mission start / mission end” coherence.

**States**

* `IDLE`
* `READY`
* `TAKEOFF_WINDOW_A`
* `TAKEOFF_WINDOW_B`
* `RUNNING`
* `RETURN_AND_LAND`
* `COMPLETE`
* `ABORT`

**Outputs**

* `/swarm/mission_state` (simple enum)
* optional “takeoff group” commands

**Important**
Even if you don’t enforce the two-takeoff-window rule in logic yet, modeling it makes behavior deterministic and easier to debug under competition constraints.

---

## 6) Traffic Manager (stub)

### Responsibility

Exist in pipeline, do nothing yet.

* Input: `/swarm/desired_goals` or `/swarm/desired_setpoints`
* Output: `/swarm/safe_goals` or `/swarm/safe_setpoints`

**v0:** passthrough + logging:

* closest pair distances
* congestion signals (many drones within radius)
* potential collisions (distance < threshold)

This gives you a hook for later.

---

# Integration Plan (what the coding agent should do)

## Step 1 — Codebase discovery (mandatory)

Agent must find:

* How drones are namespaced/identified
* Existing mission manager + explorer nodes
* Where map merging happens and the merged map topic name
* How goals are currently sent to Nav2 (action/topic/service)
* Existing state/pose publishing for each drone

## Step 2 — Add new nodes without breaking existing run

* Implement `frontier_server` and run it “shadow mode”:

  * only logs detected frontiers
* Implement `goal_allocator` in shadow mode:

  * computes assignments but does not command drones
* Implement `drone_executor` for one drone in shadow mode:

  * subscribes assignments but does not execute

## Step 3 — Switch exploration control source

* Disable per-drone frontier detection (`Nav2FrontierExplorer`) progressively:

  * start with 2 drones
  * then 5
  * then 25
* Keep Nav2 stack unchanged; only change goal source:

  * from local explorer → centralized allocator via executor

## Step 4 — Turn on full closed loop

* `mission_supervisor` drives mission phase
* drones execute centrally assigned frontiers

---

# Constraints / Non-goals (explicit)

* Do **not** modify PX4 bridge layer now.
* Do **not** redesign Nav2 configurations now.
* Do **not** implement sophisticated traffic management now.
* Do **not** optimize spawn/dispersal now.
* Focus is: **central frontier extraction + central allocation + per-drone FSM scaffolding**.

---

# What I need from you (minimal, non-invasive)

You said: “don’t ask too much; let agent discover details.” Totally.

I only need two “anchor” pieces so the agent doesn’t guess wrong:

1. **Repo entry point**: what is the main launch file you run in sim? (path/name)
2. **Node names**: confirm the actual package/node names for:

   * SwarmMissionManager
   * Nav2FrontierExplorer
   * map merger node (if you know it)

If you don’t know them, just paste the `ros2 launch ...` command you use; the agent can infer.

---

## Feedback loop (how I derived this + pitfalls + verification)

### How I got here

* You want the *strategy* of control to be refactored, not the PX4/Nav2 layers.
* Therefore, I designed a refactor that is mostly **wiring + responsibility shifts**:

  * “frontiers computed once”
  * “assignments computed once”
  * “per-drone executor consumes assignments”
* This produces modular seams that Cursor can implement incrementally.

### Pitfalls to watch for

* **TF frame mismatch**: frontier centroids in `map` frame, drone poses in `odom` → wrong goals.
* **Assignment thrash**: allocator reassigning too often.
* **Frontier flicker**: merged map updates cause clusters to split/merge.
* **ROS2 QoS**: losing `/map` or action feedback intermittently under load.

### How to verify early

* Shadow mode logs:

  * number of frontiers over time
  * assignment stability (how often each drone’s goal changes)
* 2-drone closed loop:

  * confirm goals correspond to actual frontiers
* 25-drone scale:

  * CPU drops (no 25× frontier scans)
  * less duplicate chasing

---

If you paste the launch command you run in sim, I’ll rewrite the above as a **Cursor “Implementation Checklist”** with concrete tasks like: “Create package X, add node Y, subscribe to topic discovered via grep for ‘/map’ usage, disable node Z by launch arg,” etc.—still without hardcoding your bridge/Nav2 details.
