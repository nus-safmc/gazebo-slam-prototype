#!/usr/bin/env python3
"""Adaptive autopilot publisher with pattern and exploratory modes."""

from __future__ import annotations

import math
import os
import random
from typing import Callable, Optional
from collections import deque
from heapq import nlargest

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
        self.map_frame = os.environ.get('AP_MAP_FRAME', 'robot/map')
        self.odom_frame = os.environ.get('AP_ODOM_FRAME', 'robot/odom')
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
            self._goal_history: deque[tuple[float, float]] = deque(maxlen=25)
            self._goal_min_sep = float(os.environ.get('AP_EXP_GOAL_MIN_SEP', '1.5'))
            self._goal_repeat_penalty = float(os.environ.get('AP_EXP_REPEAT_PENALTY', '8.0'))
            self._frontier_gain_w = float(os.environ.get('AP_EXP_GAIN_W', '1.0'))
            self._frontier_cost_w = float(os.environ.get('AP_EXP_COST_W', '0.35'))
            self._inflation_radius_m = float(os.environ.get('AP_EXP_INFLATE_M', '0.35'))
            self._path_step_m = float(os.environ.get('AP_EXP_PATH_STEP_M', '0.35'))
            self._lookahead_m = float(os.environ.get('AP_EXP_LOOKAHEAD_M', '1.0'))
            self._waypoint_reached_m = float(os.environ.get('AP_EXP_WP_REACHED_M', '0.4'))
            self._planner = os.environ.get('AP_EXP_PLANNER', 'frontier').lower()

            # Arena bounds (map frame). Defaults match the 20m×20m playfield with margin.
            self._arena_enabled = os.environ.get('AP_EXP_ARENA_ENABLE', '1').lower() in (
                '1',
                'true',
                'yes',
            )
            self._arena_min_x = float(os.environ.get('AP_EXP_ARENA_MIN_X', '-9.4'))
            self._arena_max_x = float(os.environ.get('AP_EXP_ARENA_MAX_X', '9.4'))
            self._arena_min_y = float(os.environ.get('AP_EXP_ARENA_MIN_Y', '-9.4'))
            self._arena_max_y = float(os.environ.get('AP_EXP_ARENA_MAX_Y', '9.4'))

            # Breadth-first (coarse-to-fine) exploration: ignore tiny frontier "holes" early
            # and prefer larger moves; relax constraints as map coverage increases.
            self._breadth_first = os.environ.get('AP_EXP_BREADTH_FIRST', '1').lower() in (
                '1',
                'true',
                'yes',
            )
            self._breadth_cov_lo = float(os.environ.get('AP_EXP_BREADTH_COV_LO', '0.10'))
            self._breadth_cov_hi = float(os.environ.get('AP_EXP_BREADTH_COV_HI', '0.75'))
            self._breadth_min_frontier_m_start = float(
                os.environ.get('AP_EXP_BREADTH_MIN_FRONTIER_M_START', '2.0')
            )
            self._breadth_min_frontier_m_end = float(
                os.environ.get('AP_EXP_BREADTH_MIN_FRONTIER_M_END', '0.4')
            )
            self._breadth_min_cost_m_start = float(
                os.environ.get('AP_EXP_BREADTH_MIN_COST_START', '2.5')
            )
            self._breadth_min_cost_m_end = float(
                os.environ.get('AP_EXP_BREADTH_MIN_COST_END', '0.8')
            )
            self._breadth_log_period = float(os.environ.get('AP_EXP_BREADTH_LOG_SEC', '8.0'))
            self._breadth_next_log = 0.0

            # Stuck detection / recovery to avoid hovering in corners forever.
            self._stuck_check_sec = float(os.environ.get('AP_STUCK_CHECK_SEC', '2.0'))
            self._stuck_min_move_m = float(os.environ.get('AP_STUCK_MIN_MOVE_M', '0.18'))
            self._stuck_cmd_speed_mps = float(os.environ.get('AP_STUCK_CMD_SPEED', '0.12'))
            self._stuck_hits = 0
            self._stuck_sample_time = self.get_clock().now()
            self._stuck_sample_x = self._pose_x
            self._stuck_sample_y = self._pose_y
            self._recovery_until = 0.0
            self._recovery_yaw = 0.0

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
                f'frames=({self.map_frame}->{self.odom_frame}) '
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
        try:
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
        except Exception as exc:
            self.get_logger().error(f'AutoPilot tick failed: {exc}')
            self._publish_stop()

    def _publish_stop(self) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

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

        if elapsed < getattr(self, '_recovery_until', 0.0):
            lin_x = -0.12 if front_min < self.explore_clear_distance else 0.0
            lin_y = 0.0
            yaw_cmd = self._recovery_yaw
            return lin_x, lin_y, yaw_cmd

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

        lin_x, lin_y, yaw_cmd = self._apply_scan_constraints(
            scan,
            lin_x,
            lin_y,
            yaw_cmd,
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
        )
        lin_x, lin_y, yaw_cmd = self._maybe_recover_stuck(
            elapsed,
            scan,
            lin_x,
            lin_y,
            yaw_cmd,
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
        )
        return lin_x, lin_y, yaw_cmd

    def _maybe_recover_stuck(
        self,
        elapsed: float,
        scan: LaserScan,
        lin_x: float,
        lin_y: float,
        yaw_cmd: float,
        *,
        front_min: float,
        left_min: float,
        right_min: float,
    ) -> tuple[float, float, float]:
        if not self._have_pose:
            return lin_x, lin_y, yaw_cmd

        # Only consider "stuck" if we're actually commanding motion.
        cmd_speed = math.hypot(lin_x, lin_y)
        if cmd_speed < self._stuck_cmd_speed_mps:
            self._stuck_hits = 0
            self._stuck_sample_time = self.get_clock().now()
            self._stuck_sample_x = self._pose_x
            self._stuck_sample_y = self._pose_y
            return lin_x, lin_y, yaw_cmd

        now = self.get_clock().now()
        dt = (now - self._stuck_sample_time).nanoseconds * 1e-9
        if dt < self._stuck_check_sec:
            return lin_x, lin_y, yaw_cmd

        moved = math.hypot(self._pose_x - self._stuck_sample_x, self._pose_y - self._stuck_sample_y)
        self._stuck_sample_time = now
        self._stuck_sample_x = self._pose_x
        self._stuck_sample_y = self._pose_y

        if moved < self._stuck_min_move_m:
            self._stuck_hits += 1
        else:
            self._stuck_hits = 0
            return lin_x, lin_y, yaw_cmd

        if self._stuck_hits < 2:
            return lin_x, lin_y, yaw_cmd

        # Recovery: clear current goal, back up, rotate toward open space.
        self._stuck_hits = 0
        self._current_path = []
        yaw_mag = abs(self.explore_turn) if self.explore_turn != 0.0 else 0.6
        yaw_sign = 1.0 if left_min >= right_min else -1.0
        self._recovery_yaw = yaw_sign * yaw_mag
        self._recovery_until = elapsed + random.uniform(1.5, 2.5)

        back = -0.15 if front_min < self.explore_clear_distance else -0.10
        return back, 0.0, self._recovery_yaw

    def _apply_scan_constraints(
        self,
        scan: LaserScan,
        lin_x: float,
        lin_y: float,
        yaw_cmd: float,
        *,
        front_min: float,
        left_min: float,
        right_min: float,
    ) -> tuple[float, float, float]:
        speed = math.hypot(lin_x, lin_y)
        if speed <= 1e-3:
            return lin_x, lin_y, yaw_cmd

        heading_deg = math.degrees(math.atan2(lin_y, lin_x))
        travel_min = self._sector_min(scan, center_deg=heading_deg, width_deg=60.0)

        # Scale safety distances with speed (time-to-collision heuristic).
        ttc_stop = float(os.environ.get('AP_EXP_TTC_STOP', '0.8'))
        ttc_slow = float(os.environ.get('AP_EXP_TTC_SLOW', '1.6'))
        avoid = max(self.explore_avoid_distance, speed * max(0.0, ttc_stop))
        clear = max(self.explore_clear_distance, speed * max(ttc_stop, ttc_slow))
        if clear <= avoid:
            clear = avoid + 0.05

        # If we're about to collide in the direction we're moving, back up (if possible)
        # and rotate toward the more open side.
        if travel_min <= avoid:
            reverse_deg = ((heading_deg + 180.0) % 360.0) - 180.0
            reverse_min = self._sector_min(scan, center_deg=reverse_deg, width_deg=60.0)
            if reverse_min > avoid:
                unit_x = lin_x / speed
                unit_y = lin_y / speed
                back_speed = min(0.18, max(0.08, 0.4 * speed))
                lin_x = -back_speed * unit_x
                lin_y = -back_speed * unit_y
            else:
                lin_x = 0.0
                lin_y = 0.0

            yaw_mag = abs(self.explore_turn) if self.explore_turn != 0.0 else 0.6
            yaw_cmd = yaw_mag if left_min >= right_min else -yaw_mag
            return lin_x, lin_y, yaw_cmd

        # Smoothly scale speed down as we approach obstacles in the travel direction.
        if travel_min < clear:
            scale = (travel_min - avoid) / (clear - avoid)
            scale = max(0.0, min(1.0, scale))
            lin_x *= scale
            lin_y *= scale

        # Prevent strafing into walls.
        if lin_y > 0.0 and left_min <= avoid:
            lin_y = 0.0
        elif lin_y < 0.0 and right_min <= avoid:
            lin_y = 0.0

        # Prevent driving forward into walls.
        if lin_x > 0.0 and front_min <= avoid:
            lin_x = 0.0

        return lin_x, lin_y, yaw_cmd

    @staticmethod
    def _sector_min(scan: LaserScan, center_deg: float, width_deg: float) -> float:
        best = float('inf')
        angle_center = math.radians(center_deg)
        half_width = math.radians(width_deg) / 2.0
        angle_min = float(scan.angle_min)
        inc = float(scan.angle_increment) if scan.angle_increment != 0.0 else 1e-6
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min:
                continue
            angle = angle_min + i * inc
            if abs(_wrap_angle(angle - angle_center)) <= half_width and r < best:
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
            if dist <= self._waypoint_reached_m:
                self._current_path.pop(0)
                continue
            break

        if not self._current_path:
            return None

        # Pure pursuit style lookahead target (in odom frame)
        lookahead = max(0.0, self._lookahead_m)
        if lookahead <= self._waypoint_reached_m:
            tx, ty = self._current_path[0]
            return tx - self._pose_x, ty - self._pose_y

        for tx, ty in self._current_path:
            dist = math.hypot(tx - self._pose_x, ty - self._pose_y)
            if dist >= lookahead:
                return tx - self._pose_x, ty - self._pose_y

        tx, ty = self._current_path[-1]
        return tx - self._pose_x, ty - self._pose_y

    def _plan_path_to_frontier(self) -> Optional[list[tuple[float, float]]]:
        if self._planner == 'legacy':
            return self._plan_path_to_frontier_legacy()
        return self._plan_path_to_frontier_frontier()

    def _plan_path_to_frontier_legacy(self) -> Optional[list[tuple[float, float]]]:
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
        map_from_odom = self._transform_components(self.map_frame, self.odom_frame)
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
            return None

        odom_from_map = self._transform_components(self.odom_frame, self.map_frame)
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

    def _plan_path_to_frontier_frontier(self) -> Optional[list[tuple[float, float]]]:
        if self._map is None or not self._have_pose:
            return None

        info = self._map.info
        width = int(info.width)
        height = int(info.height)
        if width <= 2 or height <= 2:
            return None

        res = float(info.resolution) if info.resolution > 0 else 0.0
        if res <= 0.0:
            return None

        data = self._map.data
        size = width * height
        if len(data) < size:
            return None

        origin = info.origin
        origin_x = float(origin.position.x)
        origin_y = float(origin.position.y)
        oq = origin.orientation
        siny_cosp = 2.0 * (oq.w * oq.z + oq.x * oq.y)
        cosy_cosp = 1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z)
        origin_yaw = math.atan2(siny_cosp, cosy_cosp)
        cos_o = math.cos(origin_yaw)
        sin_o = math.sin(origin_yaw)

        breadth_first = bool(getattr(self, '_breadth_first', False))
        arena_enabled = bool(getattr(self, '_arena_enabled', False))
        arena_min_x = float(getattr(self, '_arena_min_x', -math.inf))
        arena_max_x = float(getattr(self, '_arena_max_x', math.inf))
        arena_min_y = float(getattr(self, '_arena_min_y', -math.inf))
        arena_max_y = float(getattr(self, '_arena_max_y', math.inf))

        map_from_odom = self._transform_components(self.map_frame, self.odom_frame)
        if map_from_odom is None:
            return None
        map_tx, map_ty, map_yaw = map_from_odom
        robot_map_x, robot_map_y = self._apply_transform(map_tx, map_ty, map_yaw, self._pose_x, self._pose_y)

        if arena_enabled and not (
            arena_min_x <= robot_map_x <= arena_max_x and arena_min_y <= robot_map_y <= arena_max_y
        ):
            self.get_logger().warning(
                'Robot pose is outside AP_EXP_ARENA_* bounds; disabling arena filter for this plan.'
            )
            arena_enabled = False

        dx0 = robot_map_x - origin_x
        dy0 = robot_map_y - origin_y
        local_x = cos_o * dx0 + sin_o * dy0
        local_y = -sin_o * dx0 + cos_o * dy0
        start_ix = int(local_x / res)
        start_iy = int(local_y / res)
        if start_ix < 0 or start_ix >= width or start_iy < 0 or start_iy >= height:
            return None

        start_idx = start_iy * width + start_ix

        traversable = bytearray(size)
        unknown = bytearray(size)
        occupied = bytearray(size)
        free_thresh = int(self._free_thresh)
        known_cells = 0
        if arena_enabled:
            arena_area = max(0.0, (arena_max_x - arena_min_x) * (arena_max_y - arena_min_y))
            expected_total = max(1, int(round(arena_area / (res * res))))
        else:
            expected_total = max(1, size)
        for i in range(size):
            if arena_enabled:
                ix = i - (i // width) * width
                iy = i // width
                lx = (ix + 0.5) * res
                ly = (iy + 0.5) * res
                wx = origin_x + cos_o * lx - sin_o * ly
                wy = origin_y + sin_o * lx + cos_o * ly
                if not (
                    arena_min_x <= wx <= arena_max_x and arena_min_y <= wy <= arena_max_y
                ):
                    # Treat outside-of-arena as a hard obstacle so we don't "chase" unknown
                    # regions beyond the perimeter walls.
                    occupied[i] = 1
                    continue

            v = int(data[i])
            if v == -1:
                unknown[i] = 1
            elif v <= free_thresh:
                traversable[i] = 1
                known_cells += 1
            else:
                occupied[i] = 1
                known_cells += 1

        # Inflate occupied cells to avoid hugging walls.
        inflated_obstacle = bytearray(occupied)
        inflation_cells = int(math.ceil(max(0.0, self._inflation_radius_m) / res))
        if inflation_cells > 0:
            r2 = inflation_cells * inflation_cells
            for iy in range(height):
                row = iy * width
                for ix in range(width):
                    idx = row + ix
                    if not occupied[idx]:
                        continue
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        ny = iy + dy
                        if ny < 0 or ny >= height:
                            continue
                        dy2 = dy * dy
                        nrow = ny * width
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            if dy2 + dx * dx > r2:
                                continue
                            nx = ix + dx
                            if nx < 0 or nx >= width:
                                continue
                            inflated_obstacle[nrow + nx] = 1
                            traversable[nrow + nx] = 0

        if not traversable[start_idx]:
            return None

        neighbors4 = ((1, 0), (-1, 0), (0, 1), (0, -1))

        # ------------------------------------------------------------------
        # Coarse-to-fine thresholds (breadth-first exploration)
        # ------------------------------------------------------------------
        coverage = max(0.0, min(1.0, float(known_cells) / float(expected_total)))
        # t = 0 early (coarse), 1 later (fine)
        if breadth_first:
            cov_lo = float(getattr(self, '_breadth_cov_lo', 0.0))
            cov_hi = float(getattr(self, '_breadth_cov_hi', 1.0))
            if cov_hi <= cov_lo:
                t = 1.0
            else:
                t = (coverage - cov_lo) / (cov_hi - cov_lo)
            t = max(0.0, min(1.0, t))
            min_frontier_m = float(getattr(self, '_breadth_min_frontier_m_start', 0.0)) + (
                float(getattr(self, '_breadth_min_frontier_m_end', 0.0))
                - float(getattr(self, '_breadth_min_frontier_m_start', 0.0))
            ) * t
            min_cost_m = float(getattr(self, '_breadth_min_cost_m_start', 0.0)) + (
                float(getattr(self, '_breadth_min_cost_m_end', 0.0))
                - float(getattr(self, '_breadth_min_cost_m_start', 0.0))
            ) * t
        else:
            t = 1.0
            min_frontier_m = 0.0
            min_cost_m = 0.0

        if breadth_first and self._breadth_log_period > 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s >= getattr(self, '_breadth_next_log', 0.0):
                self._breadth_next_log = now_s + self._breadth_log_period
                self.get_logger().info(
                    f'Explore coverage={coverage * 100.0:.0f}% '
                    f'min_frontier={min_frontier_m:.1f}m min_cost={min_cost_m:.1f}m'
                )

        # Clearance transform: distance (cells) to nearest inflated obstacle.
        clearance_cells = [-1] * size
        cq: deque[int] = deque()
        for i in range(size):
            if inflated_obstacle[i]:
                clearance_cells[i] = 0
                cq.append(i)
        if cq:
            while cq:
                idx = cq.popleft()
                base = clearance_cells[idx]
                if base < 0:
                    continue
                ix = idx - (idx // width) * width
                iy = idx // width
                next_cost = base + 1
                for dx, dy in neighbors4:
                    nx = ix + dx
                    ny = iy + dy
                    if nx < 0 or nx >= width or ny < 0 or ny >= height:
                        continue
                    nidx = ny * width + nx
                    if clearance_cells[nidx] < 0:
                        clearance_cells[nidx] = next_cost
                        cq.append(nidx)

        # BFS on traversable cells to compute distances + parents.
        dist = [-1] * size
        parent = [-1] * size
        q: deque[int] = deque([start_idx])
        dist[start_idx] = 0

        frontier = bytearray(size)
        frontier_indices: list[int] = []

        while q:
            idx = q.popleft()
            ix = idx - (idx // width) * width
            iy = idx // width

            # Mark frontiers: reachable free cell adjacent to unknown.
            if not frontier[idx]:
                for dx, dy in neighbors4:
                    nx = ix + dx
                    ny = iy + dy
                    if nx < 0 or nx >= width or ny < 0 or ny >= height:
                        continue
                    nidx = ny * width + nx
                    if unknown[nidx]:
                        frontier[idx] = 1
                        frontier_indices.append(idx)
                        break

            next_cost = dist[idx] + 1
            for dx, dy in neighbors4:
                nx = ix + dx
                ny = iy + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                nidx = ny * width + nx
                if traversable[nidx] and dist[nidx] < 0:
                    dist[nidx] = next_cost
                    parent[nidx] = idx
                    q.append(nidx)

        if not frontier_indices:
            return None

        # Cluster frontier cells (8-connected).
        frontier_visited = bytearray(size)
        neighbors8 = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )
        clusters: list[list[int]] = []
        for idx in frontier_indices:
            if frontier_visited[idx]:
                continue
            cluster: list[int] = []
            cq: deque[int] = deque([idx])
            frontier_visited[idx] = 1
            while cq:
                cur = cq.popleft()
                cluster.append(cur)
                cx = cur - (cur // width) * width
                cy = cur // width
                for dx, dy in neighbors8:
                    nx = cx + dx
                    ny = cy + dy
                    if nx < 0 or nx >= width or ny < 0 or ny >= height:
                        continue
                    nidx = ny * width + nx
                    if frontier[nidx] and not frontier_visited[nidx]:
                        frontier_visited[nidx] = 1
                        cq.append(nidx)
            clusters.append(cluster)

        if not clusters:
            return None

        def cell_to_map_xy(cell_idx: int) -> tuple[float, float]:
            cx = cell_idx - (cell_idx // width) * width
            cy = cell_idx // width
            lx = (cx + 0.5) * res
            ly = (cy + 0.5) * res
            wx = origin_x + cos_o * lx - sin_o * ly
            wy = origin_y + sin_o * lx + cos_o * ly
            return wx, wy

        clearance_w = float(os.environ.get('AP_EXP_CLEARANCE_W', '6.0'))
        clearance_min_end = float(os.environ.get('AP_EXP_CLEARANCE_MIN', '0.6'))
        clearance_min_start_raw = os.environ.get('AP_EXP_CLEARANCE_MIN_START')
        if clearance_min_start_raw is not None and str(clearance_min_start_raw).strip():
            try:
                clearance_min_start = float(clearance_min_start_raw)
            except ValueError:
                clearance_min_start = clearance_min_end
        else:
            clearance_min_start = clearance_min_end
        clearance_min_m = clearance_min_start + (clearance_min_end - clearance_min_start) * t
        clearance_penalty_w = float(os.environ.get('AP_EXP_CLEARANCE_PENALTY', '12.0'))
        # Score clusters (information gain vs travel cost).
        def _score_clusters(
            min_frontier_len_m: float,
            min_goal_cost_m: float,
        ) -> tuple[list[tuple[float, int]], dict[int, int]]:
            scored: list[tuple[float, int]] = []
            rep_gain: dict[int, int] = {}
            for cluster in clusters:
                gain_cells = len(cluster)
                gain = float(gain_cells)
                if breadth_first and gain_cells * res < min_frontier_len_m:
                    continue

                best_rep: Optional[int] = None
                best_score = -1e18
                best_rep_world: Optional[tuple[float, float]] = None

                # Pick a representative frontier cell that is reachable AND in a wide area.
                for cand in cluster:
                    if dist[cand] < 0:
                        continue
                    cost_m = dist[cand] * res
                    if breadth_first and cost_m < min_goal_cost_m:
                        continue

                    rep_world = cell_to_map_xy(cand)
                    if arena_enabled and not (
                        arena_min_x <= rep_world[0] <= arena_max_x
                        and arena_min_y <= rep_world[1] <= arena_max_y
                    ):
                        continue

                    clear_m = (clearance_cells[cand] * res) if clearance_cells[cand] >= 0 else 0.0
                    score = self._frontier_gain_w * gain - self._frontier_cost_w * cost_m
                    score += clearance_w * clear_m
                    if clear_m < clearance_min_m:
                        score -= clearance_penalty_w * (clearance_min_m - clear_m)

                    if score > best_score:
                        best_score = score
                        best_rep = cand
                        best_rep_world = rep_world

                if best_rep is None or best_rep_world is None:
                    continue

                rep = best_rep
                rep_gain[rep] = int(gain_cells)
                score = best_score
                rep_world = best_rep_world

                if self._goal_history:
                    min_sep = min(
                        math.hypot(rep_world[0] - gx, rep_world[1] - gy)
                        for gx, gy in self._goal_history
                    )
                    if min_sep < self._goal_min_sep:
                        score -= self._goal_repeat_penalty

                scored.append((score, rep))
            return scored, rep_gain

        scored, rep_gain = _score_clusters(min_frontier_m, min_cost_m)
        if breadth_first and not scored and (min_frontier_m > 0.0 or min_cost_m > 0.0):
            # Too strict early on; relax so we always have a target to keep exploring.
            scored, rep_gain = _score_clusters(0.0, 0.0)

        if not scored:
            return None

        # Try best few candidates; skip those with broken parents (shouldn't happen).
        candidates = [rep for _, rep in nlargest(10, scored, key=lambda t: t[0])]

        odom_from_map = self._transform_components(self.odom_frame, self.map_frame)
        if odom_from_map is None:
            return None
        odom_tx, odom_ty, odom_yaw = odom_from_map

        step_cells = max(1, int(math.ceil(max(0.05, self._path_step_m) / res)))

        for rep in candidates:
            path_cells: list[int] = []
            cur = rep
            while cur != start_idx and cur != -1:
                path_cells.append(cur)
                cur = parent[cur]
            if cur == -1:
                continue
            path_cells.reverse()
            if not path_cells:
                continue

            sampled = path_cells[::step_cells]
            if sampled[-1] != path_cells[-1]:
                sampled.append(path_cells[-1])

            waypoints: list[tuple[float, float]] = []
            for cell in sampled:
                wx, wy = cell_to_map_xy(cell)
                x_odom, y_odom = self._apply_transform(odom_tx, odom_ty, odom_yaw, wx, wy)
                waypoints.append((x_odom, y_odom))

            rep_world = cell_to_map_xy(rep)
            self._goal_history.append(rep_world)
            self._last_map_target = rep_world
            self.get_logger().info(
                f'Frontier goal: ({rep_world[0]:.2f}, {rep_world[1]:.2f}) '
                f'cost={dist[rep] * res:.1f}m gain={rep_gain.get(rep, 0)}'
            )
            return waypoints

        return None

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
