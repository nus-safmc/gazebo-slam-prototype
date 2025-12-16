#!/usr/bin/env python3
"""Frontier exploration node that drives Nav2 with NavigateToPose goals.

This node selects frontier clusters (free cells adjacent to unknown) from an
OccupancyGrid map and sends sequential NavigateToPose goals to Nav2.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass(frozen=True)
class MapMeta:
    width: int
    height: int
    res: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def cos_o(self) -> float:
        return math.cos(self.origin_yaw)

    @property
    def sin_o(self) -> float:
        return math.sin(self.origin_yaw)

    def cell_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        lx = (ix + 0.5) * self.res
        ly = (iy + 0.5) * self.res
        wx = self.origin_x + self.cos_o * lx - self.sin_o * ly
        wy = self.origin_y + self.sin_o * lx + self.cos_o * ly
        return wx, wy

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        dx = wx - self.origin_x
        dy = wy - self.origin_y
        lx = self.cos_o * dx + self.sin_o * dy
        ly = -self.sin_o * dx + self.cos_o * dy
        return int(lx / self.res), int(ly / self.res)


class Nav2FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('nav2_frontier_explorer')

        self.map_topic = str(self.declare_parameter('map_topic', '/map').value)
        self.goal_frame = str(self.declare_parameter('goal_frame', 'robot/map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'robot/base_link').value)
        self.free_thresh = int(self.declare_parameter('free_threshold', 30).value)
        self.occupied_thresh = int(self.declare_parameter('occupied_threshold', 65).value)
        self.min_cluster = int(self.declare_parameter('min_frontier_cluster_size', 6).value)
        self.goal_offset_m = float(self.declare_parameter('goal_offset_m', 0.6).value)
        self.goal_timeout = float(self.declare_parameter('goal_timeout_sec', 25.0).value)
        self.replan_period = float(self.declare_parameter('replan_period_sec', 2.0).value)
        self.goal_min_sep = float(self.declare_parameter('goal_min_separation_m', 1.5).value)
        self.repeat_penalty = float(self.declare_parameter('repeat_penalty', 50.0).value)
        self.gain_weight = float(self.declare_parameter('gain_weight', 1.0).value)
        self.cost_weight = float(self.declare_parameter('cost_weight', 0.35).value)
        self.clearance_weight = float(self.declare_parameter('clearance_weight', 6.0).value)
        self.clearance_min_m = float(self.declare_parameter('clearance_min_m', 0.6).value)
        self.clearance_penalty = float(self.declare_parameter('clearance_penalty', 12.0).value)

        # Hard arena bounds (map frame) to prevent "chasing" unknown space outside perimeter walls.
        self.arena_enabled = bool(self.declare_parameter('arena_enabled', True).value)
        self.arena_min_x = float(self.declare_parameter('arena_min_x', -9.4).value)
        self.arena_max_x = float(self.declare_parameter('arena_max_x', 9.4).value)
        self.arena_min_y = float(self.declare_parameter('arena_min_y', -9.4).value)
        self.arena_max_y = float(self.declare_parameter('arena_max_y', 9.4).value)

        # Breadth-first (coarse-to-fine) exploration thresholds.
        self.breadth_first = bool(self.declare_parameter('breadth_first', True).value)
        self.breadth_cov_lo = float(self.declare_parameter('breadth_cov_lo', 0.10).value)
        self.breadth_cov_hi = float(self.declare_parameter('breadth_cov_hi', 0.75).value)
        self.breadth_min_frontier_m_start = float(
            self.declare_parameter('breadth_min_frontier_m_start', 2.0).value
        )
        self.breadth_min_frontier_m_end = float(
            self.declare_parameter('breadth_min_frontier_m_end', 0.4).value
        )
        self.breadth_min_goal_dist_start = float(
            self.declare_parameter('breadth_min_goal_dist_start', 2.5).value
        )
        self.breadth_min_goal_dist_end = float(
            self.declare_parameter('breadth_min_goal_dist_end', 0.8).value
        )
        # Optional: be stricter about wide-open space early, then relax.
        self.clearance_min_start = float(
            self.declare_parameter('clearance_min_start', self.clearance_min_m).value
        )
        self._breadth_log_period = float(self.declare_parameter('breadth_log_sec', 8.0).value)
        self._breadth_next_log = 0.0

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._result_future = None
        self._sent_time: Optional[Time] = None

        self._goal_history: deque[tuple[float, float]] = deque(maxlen=30)
        self._last_plan = self.get_clock().now()
        self._arena_next_warn = 0.0

        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Nav2 frontier explorer online (map={self.map_topic}, frame={self.goal_frame}).'
        )

    # ------------------------------------------------------------------
    # ROS callbacks / helpers
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _lookup_robot_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.goal_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    # ------------------------------------------------------------------
    # Frontier selection
    # ------------------------------------------------------------------

    @staticmethod
    def _idx(ix: int, iy: int, width: int) -> int:
        return iy * width + ix

    def _extract_meta(self, msg: OccupancyGrid) -> Optional[MapMeta]:
        info = msg.info
        width = int(info.width)
        height = int(info.height)
        res = float(info.resolution)
        if width <= 0 or height <= 0 or res <= 0.0:
            return None
        o = info.origin
        yaw = _yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
        return MapMeta(
            width=width,
            height=height,
            res=res,
            origin_x=float(o.position.x),
            origin_y=float(o.position.y),
            origin_yaw=yaw,
        )

    def _find_frontier_goal(
        self,
        msg: OccupancyGrid,
        robot_xy: tuple[float, float],
    ) -> Optional[tuple[float, float]]:
        meta = self._extract_meta(msg)
        if meta is None:
            return None
        size = meta.width * meta.height
        data = msg.data
        if len(data) < size:
            return None

        arena_enabled = bool(getattr(self, 'arena_enabled', False))
        arena_min_x = float(getattr(self, 'arena_min_x', -math.inf))
        arena_max_x = float(getattr(self, 'arena_max_x', math.inf))
        arena_min_y = float(getattr(self, 'arena_min_y', -math.inf))
        arena_max_y = float(getattr(self, 'arena_max_y', math.inf))
        robot_x, robot_y = robot_xy
        if arena_enabled and not (
            arena_min_x <= robot_x <= arena_max_x and arena_min_y <= robot_y <= arena_max_y
        ):
            # Keep the arena filter enabled so we don't chase unknown space outside the perimeter.
            # If the robot slightly clips outside bounds (e.g. after a wall contact), clamp the
            # pose for scoring purposes rather than disabling the bounds entirely.
            clamped_x = min(max(robot_x, arena_min_x), arena_max_x)
            clamped_y = min(max(robot_y, arena_min_y), arena_max_y)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s >= getattr(self, '_arena_next_warn', 0.0):
                self._arena_next_warn = now_s + 5.0
                self.get_logger().warning(
                    f'Robot pose ({robot_x:.2f},{robot_y:.2f}) is outside arena_* bounds; '
                    f'clamping to ({clamped_x:.2f},{clamped_y:.2f}) for planning.'
                )
            robot_x, robot_y = clamped_x, clamped_y

        unknown = bytearray(size)
        free = bytearray(size)
        obstacle = bytearray(size)

        known_cells = 0
        if arena_enabled:
            arena_area = max(0.0, (arena_max_x - arena_min_x) * (arena_max_y - arena_min_y))
            expected_total = max(1, int(round(arena_area / (meta.res * meta.res))))
        else:
            expected_total = max(1, size)

        cos_o = math.cos(meta.origin_yaw)
        sin_o = math.sin(meta.origin_yaw)
        for i in range(size):
            if arena_enabled:
                ix = i - (i // meta.width) * meta.width
                iy = i // meta.width
                lx = (ix + 0.5) * meta.res
                ly = (iy + 0.5) * meta.res
                wx = meta.origin_x + cos_o * lx - sin_o * ly
                wy = meta.origin_y + sin_o * lx + cos_o * ly
                if not (
                    arena_min_x <= wx <= arena_max_x and arena_min_y <= wy <= arena_max_y
                ):
                    obstacle[i] = 1
                    continue

            v = int(data[i])
            if v == -1:
                unknown[i] = 1
                continue

            known_cells += 1
            if v <= self.free_thresh:
                free[i] = 1
            elif v >= self.occupied_thresh:
                obstacle[i] = 1

        coverage = max(0.0, min(1.0, float(known_cells) / float(expected_total)))
        if self.breadth_first:
            if self.breadth_cov_hi <= self.breadth_cov_lo:
                t = 1.0
            else:
                t = (coverage - self.breadth_cov_lo) / (self.breadth_cov_hi - self.breadth_cov_lo)
            t = max(0.0, min(1.0, t))
            min_frontier_m = self.breadth_min_frontier_m_start + (
                self.breadth_min_frontier_m_end - self.breadth_min_frontier_m_start
            ) * t
            min_goal_dist_m = self.breadth_min_goal_dist_start + (
                self.breadth_min_goal_dist_end - self.breadth_min_goal_dist_start
            ) * t
            clearance_min_m = self.clearance_min_start + (self.clearance_min_m - self.clearance_min_start) * t
        else:
            min_frontier_m = 0.0
            min_goal_dist_m = 0.0
            clearance_min_m = self.clearance_min_m

        if self.breadth_first and self._breadth_log_period > 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s >= getattr(self, '_breadth_next_log', 0.0):
                self._breadth_next_log = now_s + self._breadth_log_period
                self.get_logger().info(
                    f'Explore coverage={coverage * 100.0:.0f}% '
                    f'min_frontier={min_frontier_m:.1f}m min_goal_dist={min_goal_dist_m:.1f}m'
                )

        neighbors4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
        neighbors8 = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )

        frontier = bytearray(size)
        frontier_indices: list[int] = []
        for iy in range(1, meta.height - 1):
            row = iy * meta.width
            for ix in range(1, meta.width - 1):
                idx = row + ix
                if not free[idx]:
                    continue
                for dx, dy in neighbors4:
                    nidx = (iy + dy) * meta.width + (ix + dx)
                    if unknown[nidx]:
                        frontier[idx] = 1
                        frontier_indices.append(idx)
                        break

        if not frontier_indices:
            return None

        visited = bytearray(size)
        clusters: list[list[int]] = []
        for seed in frontier_indices:
            if visited[seed]:
                continue
            cluster: list[int] = []
            q: deque[int] = deque([seed])
            visited[seed] = 1
            while q:
                cur = q.popleft()
                cluster.append(cur)
                cx = cur - (cur // meta.width) * meta.width
                cy = cur // meta.width
                for dx, dy in neighbors8:
                    nx = cx + dx
                    ny = cy + dy
                    if nx <= 0 or nx >= meta.width - 1 or ny <= 0 or ny >= meta.height - 1:
                        continue
                    nidx = ny * meta.width + nx
                    if frontier[nidx] and not visited[nidx]:
                        visited[nidx] = 1
                        q.append(nidx)
            if len(cluster) >= self.min_cluster:
                clusters.append(cluster)

        if not clusters:
            return None

        # Clearance transform: distance (cells) to nearest obstacle. (Unknown doesn't count.)
        clearance_cells = [-1] * size
        cq: deque[int] = deque()
        for i in range(size):
            if obstacle[i]:
                clearance_cells[i] = 0
                cq.append(i)
        if cq:
            while cq:
                idx = cq.popleft()
                base = clearance_cells[idx]
                if base < 0:
                    continue
                ix = idx - (idx // meta.width) * meta.width
                iy = idx // meta.width
                next_cost = base + 1
                for dx, dy in neighbors4:
                    nx = ix + dx
                    ny = iy + dy
                    if nx <= 0 or nx >= meta.width - 1 or ny <= 0 or ny >= meta.height - 1:
                        continue
                    nidx = ny * meta.width + nx
                    if clearance_cells[nidx] < 0:
                        clearance_cells[nidx] = next_cost
                        cq.append(nidx)

        offset_cells = max(1, int(math.ceil(self.goal_offset_m / meta.res)))

        def _pick_goal(
            min_frontier_len_m: float,
            min_goal_dist: float,
            clearance_min: float,
        ) -> Optional[tuple[float, float]]:
            best_score = -1e12
            best_goal: Optional[tuple[float, float]] = None

            for cluster in clusters:
                if len(cluster) * meta.res < min_frontier_len_m:
                    continue

                # Compute centroid in grid coords.
                sx = 0.0
                sy = 0.0
                for c in cluster:
                    sx += c - (c // meta.width) * meta.width
                    sy += c // meta.width
                cx = sx / len(cluster)
                cy = sy / len(cluster)

                # Choose representative frontier cell closest to centroid.
                rep = min(
                    cluster,
                    key=lambda i: (
                        (i - (i // meta.width) * meta.width - cx) ** 2
                        + (i // meta.width - cy) ** 2
                    ),
                )
                rep_x = rep - (rep // meta.width) * meta.width
                rep_y = rep // meta.width

                # Estimate direction away from unknown and step inward.
                vx = 0.0
                vy = 0.0
                for dx, dy in neighbors4:
                    nidx = (rep_y + dy) * meta.width + (rep_x + dx)
                    if unknown[nidx]:
                        vx -= dx
                        vy -= dy
                if vx == 0.0 and vy == 0.0:
                    # Fallback: move toward cluster centroid.
                    vx = cx - rep_x
                    vy = cy - rep_y
                norm = math.hypot(vx, vy)
                if norm <= 1e-6:
                    continue
                vx /= norm
                vy /= norm

                goal_ix = int(round(rep_x + vx * offset_cells))
                goal_iy = int(round(rep_y + vy * offset_cells))
                goal_ix = max(1, min(meta.width - 2, goal_ix))
                goal_iy = max(1, min(meta.height - 2, goal_iy))

                goal_idx = goal_iy * meta.width + goal_ix
                if not free[goal_idx]:
                    # Search a small neighborhood for a free cell.
                    found = False
                    for radius in range(1, 6):
                        for dx in range(-radius, radius + 1):
                            for dy in range(-radius, radius + 1):
                                nx = goal_ix + dx
                                ny = goal_iy + dy
                                if (
                                    nx <= 0
                                    or nx >= meta.width - 1
                                    or ny <= 0
                                    or ny >= meta.height - 1
                                ):
                                    continue
                                nidx = ny * meta.width + nx
                                if free[nidx]:
                                    goal_ix, goal_iy = nx, ny
                                    goal_idx = nidx
                                    found = True
                                    break
                            if found:
                                break
                        if found:
                            break
                    if not found:
                        continue

                goal_x, goal_y = meta.cell_to_world(goal_ix, goal_iy)
                dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
                if dist < min_goal_dist:
                    continue

                gain = float(len(cluster))
                score = self.gain_weight * gain - self.cost_weight * dist
                if clearance_cells[goal_idx] >= 0:
                    clearance_m = clearance_cells[goal_idx] * meta.res
                    score += self.clearance_weight * clearance_m
                    if clearance_m < clearance_min:
                        score -= self.clearance_penalty * (clearance_min - clearance_m)
                if self._goal_history:
                    min_sep = min(
                        math.hypot(goal_x - gx, goal_y - gy) for gx, gy in self._goal_history
                    )
                    if min_sep < self.goal_min_sep:
                        score -= self.repeat_penalty

                if score > best_score:
                    best_score = score
                    best_goal = (goal_x, goal_y)

            return best_goal

        goal = _pick_goal(min_frontier_m, min_goal_dist_m, clearance_min_m)
        if self.breadth_first and goal is None and (min_frontier_m > 0.0 or min_goal_dist_m > 0.0):
            goal = _pick_goal(0.0, 0.0, self.clearance_min_m)
        return goal

    # ------------------------------------------------------------------
    # Nav2 integration
    # ------------------------------------------------------------------

    def _send_goal(self, gx: float, gy: float, yaw: float) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._sent_time = self.get_clock().now()
        self._goal_history.append((gx, gy))
        self.get_logger().info(f'Sending goal: ({gx:.2f}, {gy:.2f}) yaw={yaw:.2f}')

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected goal.')
            self._goal_handle = None
            self._result_future = None
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning('Goal aborted by Nav2.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Goal canceled.')
        else:
            self.get_logger().info(f'Goal result status={status}')
        self._goal_handle = None
        self._result_future = None
        self._sent_time = None

    def _cancel_goal(self) -> None:
        if self._goal_handle is None:
            return
        self.get_logger().warning('Canceling current goal (timeout).')
        self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None
        self._sent_time = None

    def _tick(self) -> None:
        if self._map is None:
            return
        if not self._nav.wait_for_server(timeout_sec=0.0):
            return

        # Cancel runaway goals / avoid spamming goals while one is in-flight.
        if self._sent_time is not None:
            elapsed = (self.get_clock().now() - self._sent_time).nanoseconds * 1e-9
            if self._goal_handle is None:
                # Goal request still pending acceptance.
                if elapsed > 3.0:
                    self.get_logger().warning('Goal not accepted yet; retrying.')
                    self._sent_time = None
                return
            if elapsed > self.goal_timeout:
                self._cancel_goal()
            return

        now = self.get_clock().now()
        if (now - self._last_plan).nanoseconds * 1e-9 < self.replan_period:
            return
        self._last_plan = now

        pose = self._lookup_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        goal = self._find_frontier_goal(self._map, (rx, ry))
        if goal is None:
            return

        gx, gy = goal
        yaw = math.atan2(gy - ry, gx - rx)
        self._send_goal(gx, gy, yaw)


def main() -> None:
    rclpy.init()
    node = Nav2FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['Nav2FrontierExplorer', 'main']
