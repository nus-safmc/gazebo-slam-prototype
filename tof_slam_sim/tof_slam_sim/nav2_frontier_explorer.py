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

        self.declare_parameter('use_sim_time', True)
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

        unknown = bytearray(size)
        free = bytearray(size)
        for i in range(size):
            v = int(data[i])
            if v == -1:
                unknown[i] = 1
            elif v <= self.free_thresh:
                free[i] = 1

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

        robot_x, robot_y = robot_xy
        best_score = -1e12
        best_goal: Optional[tuple[float, float]] = None

        offset_cells = max(1, int(math.ceil(self.goal_offset_m / meta.res)))

        for cluster in clusters:
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
                key=lambda i: (i - (i // meta.width) * meta.width - cx) ** 2 + (i // meta.width - cy) ** 2,
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
                            if nx <= 0 or nx >= meta.width - 1 or ny <= 0 or ny >= meta.height - 1:
                                continue
                            nidx = ny * meta.width + nx
                            if free[nidx]:
                                goal_ix, goal_iy = nx, ny
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
            gain = float(len(cluster))

            score = self.gain_weight * gain - self.cost_weight * dist
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
