#!/usr/bin/env python3
"""Adaptive autopilot publisher with pattern and exploratory modes."""

from __future__ import annotations

import math
import os
import random
from typing import Callable, Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from rclpy.time import Time


def qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class AutoPilot(Node):
    """Publish Twist commands for either patterned or exploratory motion."""

    def __init__(self) -> None:
        super().__init__('auto_pilot')

        # Topic & QoS
        topic = os.environ.get('AP_TOPIC', '/cmd_vel')
        self.pub = self.create_publisher(Twist, topic, qos_reliable(10))

        # Initial kinematics (overridden per mode as needed)
        lin_x = float(os.environ.get('AP_LIN_X', '0.3'))
        lin_y = float(os.environ.get('AP_LIN_Y', '0.0'))
        ang_z = float(os.environ.get('AP_ANG_Z', '0.3'))
        self.msg = Twist()
        self.msg.linear.x = lin_x
        self.msg.linear.y = lin_y
        self.msg.linear.z = 0.0
        self.msg.angular.z = ang_z

        # State shared by both modes
        self.mode = os.environ.get('AP_MODE', 'pattern').lower()
        self._pattern: Optional[Callable[[float], tuple[float, float, float]]] = None
        self._start_time = self.get_clock().now()
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._have_pose = False
        self._current_path: list[tuple[float, float]] = []
        self._path_replan_period = float(os.environ.get('AP_EXP_REPLAN_SEC', '2.5'))
        self._path_last_plan = 0.0
        self._sensor_range = 0.0

        # Optional altitude control (off by default to avoid rmw issues)
        self.alt_enable = os.environ.get('AP_ALT_ENABLE', '0').lower() in ('1', 'true', 'yes')
        self.current_alt: Optional[float] = None
        self.alt_target = float(os.environ.get('AP_ALT_TARGET', '1.5'))
        self.alt_kp = float(os.environ.get('AP_ALT_KP', '0.8'))
        self.alt_deadband = float(os.environ.get('AP_ALT_DEADBAND', '0.05'))
        self.max_vert_speed = float(os.environ.get('AP_LIN_Z_MAX', '0.2'))

        odom_topic = os.environ.get('AP_ODOM', '/odom')
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_reliable(10))

        # Rate
        hz = float(os.environ.get('AP_RATE', '10.0'))
        period = 1.0 / hz if hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self._tick)

        # TF buffer for frame transforms (map <-> odom)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # Mode-specific configuration
        if self.mode == 'explore':
            scan_topic = os.environ.get('AP_SCAN_TOPIC', '/scan_merged')
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._last_scan: Optional[LaserScan] = None
            self.create_subscription(LaserScan, scan_topic, self._scan_cb, sensor_qos)

            self.explore_forward = float(os.environ.get('AP_EXP_FORWARD', '0.45'))
            self.explore_strafe = float(os.environ.get('AP_EXP_STRAFE', '0.18'))
            self.explore_turn = float(os.environ.get('AP_EXP_TURN', '0.6'))
            self.explore_clear_distance = float(os.environ.get('AP_EXP_CLEAR', '1.1'))
            self.explore_avoid_distance = float(os.environ.get('AP_EXP_AVOID', '0.6'))
            self._explore_bias_end = 0.0
            self._explore_next_bias = 3.0
            self._explore_bias_yaw = 0.0

            self._map: Optional[OccupancyGrid] = None
            self._map_stride = max(1, int(os.environ.get('AP_EXP_MAP_STRIDE', '4')))
            self._unknown_min = float(os.environ.get('AP_EXP_UNKNOWN_MIN', '2.5'))
            self._unknown_max = float(os.environ.get('AP_EXP_UNKNOWN_MAX', '12.0'))
            self._free_thresh = int(os.environ.get('AP_EXP_FREE_THRESH', '30'))
            self._range_factor = float(os.environ.get('AP_EXP_RANGE_FACTOR', '0.95'))
            self._last_map_target: Optional[tuple[float, float]] = None

            map_topic = os.environ.get('AP_MAP_TOPIC', '/map')
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

            self.get_logger().info(
                f'Exploration mode active (scan={scan_topic}, map={map_topic}) '
                f'forward={self.explore_forward:.2f} turn={self.explore_turn:.2f}'
            )
        else:
            self._pattern = self._build_pattern()
            pattern_name = os.environ.get('AP_PATTERN', 'lissajous').lower()
            self.get_logger().info(
                f'Publishing Twist on {topic} @ {hz:.1f} Hz '
                f'pattern={pattern_name} alt_ctrl={self.alt_enable}'
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_alt = float(msg.pose.pose.position.z)
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._have_pose = True

    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_scan = msg
        rng = msg.range_max
        self._sensor_range = rng if math.isfinite(rng) else 0.0

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    # ------------------------------------------------------------------
    # Core control loop
    # ------------------------------------------------------------------

    def _altitude_command(self) -> float:
        if not self.alt_enable:
            return 0.0
        if self.current_alt is None:
            return min(self.max_vert_speed, 0.2)
        error = self.alt_target - self.current_alt
        if abs(error) <= self.alt_deadband:
            return 0.0
        cmd = self.alt_kp * error
        return max(-self.max_vert_speed, min(self.max_vert_speed, cmd))

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if self.mode == 'explore':
            lin_x, lin_y, ang_z = self._explore_command(elapsed)
        else:
            lin_x, lin_y, ang_z = self._pattern(elapsed) if self._pattern else (
                self.msg.linear.x,
                self.msg.linear.y,
                self.msg.angular.z,
            )
        self.msg.linear.x = lin_x
        self.msg.linear.y = lin_y
        self.msg.angular.z = ang_z
        self.msg.linear.z = self._altitude_command()
        self.pub.publish(self.msg)

    # ------------------------------------------------------------------
    # Pattern mode
    # ------------------------------------------------------------------

    def _build_pattern(self) -> Callable[[float], tuple[float, float, float]]:
        pattern = os.environ.get('AP_PATTERN', 'lissajous').lower()
        base_lin_x = self.msg.linear.x
        base_lin_y = self.msg.linear.y
        base_ang_z = self.msg.angular.z

        if pattern in ('constant', 'steady'):
            def _const(_: float) -> tuple[float, float, float]:
                return base_lin_x, base_lin_y, base_ang_z

            return _const

        amp_x = float(os.environ.get('AP_PATTERN_AMP_X', '0.6'))
        amp_y = float(os.environ.get('AP_PATTERN_AMP_Y', '0.45'))
        freq_x = float(os.environ.get('AP_PATTERN_FREQ_X', '0.20'))
        freq_y = float(os.environ.get('AP_PATTERN_FREQ_Y', '0.32'))
        phase = float(os.environ.get('AP_PATTERN_PHASE', '1.0472'))
        yaw_gain = float(os.environ.get('AP_PATTERN_YAW_GAIN', '0.4'))

        self.get_logger().info(
            'AutoPilot lissajous pattern '
            f'amp=({amp_x:.2f},{amp_y:.2f}) freq=({freq_x:.2f},{freq_y:.2f})'
        )

        def _lissajous(t: float) -> tuple[float, float, float]:
            vx = amp_x * math.sin(2.0 * math.pi * freq_x * t)
            vy = amp_y * math.sin(2.0 * math.pi * freq_y * t + phase)
            yaw = yaw_gain * math.sin(2.0 * math.pi * freq_y * t)
            return vx, vy, yaw

        return _lissajous

    # ------------------------------------------------------------------
    # Exploration mode helpers
    # ------------------------------------------------------------------

    def _explore_command(self, elapsed: float) -> tuple[float, float, float]:
        # Default gentle turn if sensors not ready yet
        if self._last_scan is None:
            return 0.0, 0.0, 0.35

        scan = self._last_scan
        front_min = self._sector_min(scan, center_deg=0.0, width_deg=60.0)
        left_min = self._sector_min(scan, center_deg=75.0, width_deg=70.0)
        right_min = self._sector_min(scan, center_deg=-75.0, width_deg=70.0)

        yaw_cmd = 0.0
        lin_x = self.explore_forward
        lin_y = 0.0

        target_vec = self._ensure_path_and_target(elapsed)

        if front_min < self.explore_avoid_distance:
            lin_x = -0.1
            if left_min > right_min:
                yaw_cmd = self.explore_turn
                lin_y = self.explore_strafe
            else:
                yaw_cmd = -self.explore_turn
                lin_y = -self.explore_strafe
        elif front_min < self.explore_clear_distance:
            lin_x = 0.15
            if left_min > right_min:
                yaw_cmd = self.explore_turn * 0.7
                lin_y = self.explore_strafe * 0.5
            else:
                yaw_cmd = -self.explore_turn * 0.7
                lin_y = -self.explore_strafe * 0.5
        elif target_vec is not None:
            dx, dy = target_vec
            cos_yaw = math.cos(self._pose_yaw)
            sin_yaw = math.sin(self._pose_yaw)
            body_x = cos_yaw * dx + sin_yaw * dy
            body_y = -sin_yaw * dx + cos_yaw * dy
            heading_error = math.atan2(body_y, body_x)
            yaw_cmd = max(-self.explore_turn, min(self.explore_turn, heading_error))
            desired_x = max(-self.explore_forward, min(self.explore_forward, body_x))
            desired_y = max(-self.explore_strafe, min(self.explore_strafe, body_y))
            lin_x = desired_x
            lin_y = desired_y
            dist = math.hypot(dx, dy)
            if dist < 0.75:
                lin_x *= dist / 0.75
                lin_y *= dist / 0.75
        else:
            now = elapsed
            if now >= self._explore_next_bias:
                self._explore_bias_yaw = random.uniform(
                    -self.explore_turn * 0.6, self.explore_turn * 0.6
                )
                duration = random.uniform(2.0, 4.0)
                self._explore_bias_end = now + duration
                self._explore_next_bias = now + duration + random.uniform(3.0, 6.0)
            if now < self._explore_bias_end:
                yaw_cmd = self._explore_bias_yaw
                lin_y = self.explore_strafe * math.sin(now)

        return lin_x, lin_y, yaw_cmd

    @staticmethod
    def _sector_min(scan: LaserScan, center_deg: float, width_deg: float) -> float:
        angle_center = math.radians(center_deg)
        half_width = math.radians(width_deg) / 2.0
        start = angle_center - half_width
        end = angle_center + half_width

        idx_start = max(0, int((start - scan.angle_min) / scan.angle_increment))
        idx_end = min(len(scan.ranges) - 1, int((end - scan.angle_min) / scan.angle_increment))
        if idx_end < idx_start:
            idx_start, idx_end = idx_end, idx_start

        best = float('inf')
        for i in range(idx_start, idx_end + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > scan.range_min:
                if r < best:
                    best = r
        if not math.isfinite(best):
            best = scan.range_max if math.isfinite(scan.range_max) else 10.0
        return best

    def _ensure_path_and_target(self, elapsed: float) -> Optional[tuple[float, float]]:
        if (
            not self._current_path
            or elapsed - self._path_last_plan >= self._path_replan_period
        ):
            path = self._plan_path_to_frontier()
            if path:
                self._current_path = path
                self._path_last_plan = elapsed
            else:
                self._current_path = []

        while self._current_path:
            tx, ty = self._current_path[0]
            dist = math.hypot(tx - self._pose_x, ty - self._pose_y)
            if dist <= 0.4:
                self._current_path.pop(0)
                continue
            return tx - self._pose_x, ty - self._pose_y

        return None

    def _plan_path_to_frontier(self) -> Optional[list[tuple[float, float]]]:
        if self._map is None or not self._have_pose:
            return None

        info = self._map.info
        data = self._map.data
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        stride = self._map_stride

        # Determine robot position in map frame
        map_from_odom = self._transform_components('map', 'odom')
        if map_from_odom is None:
            return None
        map_tx, map_ty, map_yaw = map_from_odom
        robot_map_x, robot_map_y = self._apply_transform(map_tx, map_ty, map_yaw, self._pose_x, self._pose_y)
        robot_ix = int((robot_map_x - origin_x) / res)
        robot_iy = int((robot_map_y - origin_y) / res)
        if robot_ix < 0 or robot_ix >= width or robot_iy < 0 or robot_iy >= height:
            return None

        visited: set[tuple[int, int]] = {(robot_ix, robot_iy)}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        q: deque[tuple[int, int]] = deque([(robot_ix, robot_iy)])

        best_parent: Optional[tuple[int, int]] = None
        best_frontier_world: Optional[tuple[float, float]] = None
        best_score = -1.0

        limit = min(width * height, 60000)

        while q and len(visited) <= limit:
            ix, iy = q.popleft()
            for nx, ny in ((ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)):
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                n_idx = ny * width + nx
                if n_idx >= len(data):
                    continue
                cell = data[n_idx]
                target_x = origin_x + (nx + 0.5) * res
                target_y = origin_y + (ny + 0.5) * res
                dist = math.hypot(target_x - robot_map_x, target_y - robot_map_y)
                if cell == -1:
                    if (
                        self._sensor_range > 0.0
                        and dist > self._sensor_range * self._range_factor
                        and dist <= self._unknown_max
                    ):
                        # treat as traversable (beyond current sensor horizon)
                        if (nx, ny) in visited:
                            continue
                        visited.add((nx, ny))
                        parent[(nx, ny)] = (ix, iy)
                        q.append((nx, ny))
                        continue
                    if dist < self._unknown_min or dist > self._unknown_max:
                        continue
                    if (
                        self._last_map_target is not None
                        and math.hypot(
                            target_x - self._last_map_target[0],
                            target_y - self._last_map_target[1],
                        )
                        < self._unknown_min
                    ):
                        continue
                    if dist > best_score:
                        best_score = dist
                        best_parent = (ix, iy)
                        best_frontier_world = (target_x, target_y)
                    continue
                if cell > self._free_thresh:
                    continue
                if (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                parent[(nx, ny)] = (ix, iy)
                q.append((nx, ny))

        if best_parent is None or best_frontier_world is None:
            self._last_map_target = None
            self._last_map_target = None
            return None

        odom_from_map = self._transform_components('odom', 'map')
        if odom_from_map is None:
            return None
        odom_tx, odom_ty, odom_yaw = odom_from_map

        path_cells: list[tuple[int, int]] = []
        cell = best_parent
        start_cell = (robot_ix, robot_iy)
        while True:
            path_cells.append(cell)
            if cell == start_cell or cell not in parent:
                break
            cell = parent[cell]
        path_cells.reverse()

        waypoints: list[tuple[float, float]] = []
        for cx, cy in path_cells:
            if (cx, cy) == start_cell:
                continue
            wx = origin_x + (cx + 0.5) * res
            wy = origin_y + (cy + 0.5) * res
            x_odom, y_odom = self._apply_transform(odom_tx, odom_ty, odom_yaw, wx, wy)
            waypoints.append((x_odom, y_odom))

        if best_frontier_world is not None:
            self._last_map_target = best_frontier_world
            fx, fy = best_frontier_world
            x_odom, y_odom = self._apply_transform(odom_tx, odom_ty, odom_yaw, fx, fy)
            waypoints.append((x_odom, y_odom))

        return waypoints if waypoints else None

    def _transform_components(self, target: str, source: str) -> Optional[tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().debug(f'Failed transform {source}->{target}: {exc}')
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return tx, ty, yaw

    @staticmethod
    def _apply_transform(tx: float, ty: float, yaw: float, x: float, y: float) -> tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        new_x = tx + cos_yaw * x - sin_yaw * y
        new_y = ty + sin_yaw * x + cos_yaw * y
        return new_x, new_y


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def main() -> None:
    rclpy.init()
    node = AutoPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['AutoPilot', 'main']
