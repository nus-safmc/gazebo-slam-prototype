#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


def _is_finite(value: float) -> bool:
    return math.isfinite(float(value))


def _quat_to_rot(w: float, x: float, y: float, z: float) -> list[list[float]]:
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z

    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    return [
        [ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz],
    ]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _yaw_from_rot_enu_flu(r: list[list[float]]) -> float:
    return math.atan2(r[1][0], r[0][0])


def _yaw_enu_from_px4_q(q_wxyz: list[float]) -> Optional[float]:
    if len(q_wxyz) != 4:
        return None
    w, x, y, z = (float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))
    if not all(_is_finite(v) for v in (w, x, y, z)):
        return None

    # PX4: q is FRD(body) -> NED(world). We want yaw in ENU about +Z (up).
    r_ned_frd = _quat_to_rot(w, x, y, z)
    r_enu_ned = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    r_frd_flu = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    r_enu_flu = _mat_mul(_mat_mul(r_enu_ned, r_ned_frd), r_frd_flu)
    return float(_yaw_from_rot_enu_flu(r_enu_flu))


@dataclass
class _Latest:
    msg: object
    stamp_ns: int


class TwistToPX4Offboard(Node):
    """Translate ROS `cmd_vel` into PX4 offboard velocity setpoints (uXRCE-DDS)."""

    def __init__(self) -> None:
        super().__init__('twist_to_px4_offboard')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('vehicle_odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('deadman_timeout_sec', 0.75)

        self.declare_parameter('target_alt_m', 1.5)
        self.declare_parameter('alt_kp', 0.8)
        self.declare_parameter('max_climb_mps', 0.8)
        self.declare_parameter('max_descend_mps', 0.6)

        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('warmup_setpoints', 20)
        self.declare_parameter('command_period_sec', 2.0)

        self.declare_parameter('px4_target_system', 1)
        self.declare_parameter('px4_target_component', 1)
        self.declare_parameter('px4_source_system', 1)
        self.declare_parameter('px4_source_component', 1)

        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value).strip() or '/cmd_vel'
        self._vehicle_odom_topic = (
            str(self.get_parameter('vehicle_odometry_topic').value).strip()
            or '/fmu/out/vehicle_odometry'
        )

        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._publish_rate_hz = max(1.0, self._publish_rate_hz)

        self._deadman_timeout_ns = int(
            float(self.get_parameter('deadman_timeout_sec').value) * 1e9
        )
        self._target_alt_m = float(self.get_parameter('target_alt_m').value)
        self._alt_kp = float(self.get_parameter('alt_kp').value)
        self._max_climb_mps = float(self.get_parameter('max_climb_mps').value)
        self._max_descend_mps = float(self.get_parameter('max_descend_mps').value)

        self._auto_arm = bool(self.get_parameter('auto_arm').value)
        self._auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self._warmup_setpoints = max(0, int(self.get_parameter('warmup_setpoints').value))
        self._command_period_ns = int(
            float(self.get_parameter('command_period_sec').value) * 1e9
        )

        self._target_system = int(self.get_parameter('px4_target_system').value)
        self._target_component = int(self.get_parameter('px4_target_component').value)
        self._source_system = int(self.get_parameter('px4_source_system').value)
        self._source_component = int(self.get_parameter('px4_source_component').value)

        qos_px4_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_px4_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_inputs = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_px4_in
        )
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_px4_in
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_px4_in
        )

        self._status: Optional[VehicleStatus] = None
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._on_status,
            qos_px4_out,
        )

        self._failsafe_flags: Optional[FailsafeFlags] = None
        self.create_subscription(
            FailsafeFlags,
            '/fmu/out/failsafe_flags',
            self._on_failsafe_flags,
            qos_px4_out,
        )

        self._latest_vehicle_odom: Optional[_Latest] = None
        self._px4_ts_us: Optional[int] = None
        self.create_subscription(
            VehicleOdometry,
            self._vehicle_odom_topic,
            self._on_vehicle_odometry,
            qos_px4_out,
        )

        self._latest_cmd: Optional[_Latest] = None
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, qos_inputs)

        self._setpoint_count = 0
        self._last_mode_cmd_ns: Optional[int] = None
        self._last_arm_cmd_ns: Optional[int] = None

        period = 1.0 / self._publish_rate_hz
        self.create_timer(period, self._tick)

        self.get_logger().info(
            'PX4 offboard bridge online: '
            f'cmd_vel={self._cmd_vel_topic} vehicle_odom={self._vehicle_odom_topic} '
            f'rate={self._publish_rate_hz:.1f}Hz target_alt={self._target_alt_m:.2f}m '
            f'auto_offboard={self._auto_offboard} auto_arm={self._auto_arm}'
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _now_us(self) -> int:
        return int(self._now_ns() // 1000)

    def _px4_now_us(self) -> int:
        if self._px4_ts_us is not None:
            return int(self._px4_ts_us)
        return self._now_us()

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _on_failsafe_flags(self, msg: FailsafeFlags) -> None:
        self._failsafe_flags = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_cmd = _Latest(msg=msg, stamp_ns=self._now_ns())

    def _on_vehicle_odometry(self, msg: VehicleOdometry) -> None:
        self._px4_ts_us = int(msg.timestamp)
        self._latest_vehicle_odom = _Latest(msg=msg, stamp_ns=self._now_ns())

    def _send_vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._px4_now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = int(self._target_system)
        msg.target_component = int(self._target_component)
        msg.source_system = int(self._source_system)
        msg.source_component = int(self._source_component)
        msg.confirmation = 0
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _maybe_send_offboard_and_arm(self, now_ns: int) -> None:
        if not self._auto_offboard and not self._auto_arm:
            return
        if self._setpoint_count < self._warmup_setpoints:
            return

        armed = False
        offboard = False
        if self._status is not None:
            armed = self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        local_pos_ok = True
        flags = self._failsafe_flags
        if flags is not None and flags.local_position_invalid_relaxed:
            local_pos_ok = False

        if self._auto_offboard and not offboard and local_pos_ok:
            if (
                self._last_mode_cmd_ns is None
                or now_ns - self._last_mode_cmd_ns >= self._command_period_ns
            ):
                # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, PX4_CUSTOM_MAIN_MODE_OFFBOARD=6.
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
                )
                self._last_mode_cmd_ns = now_ns

        if self._auto_arm and not armed and local_pos_ok:
            if (
                self._last_arm_cmd_ns is None
                or now_ns - self._last_arm_cmd_ns >= self._command_period_ns
            ):
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
                )
                self._last_arm_cmd_ns = now_ns

    def _latest_cmd_vel(self, now_ns: int) -> Twist:
        cmd = self._latest_cmd
        if cmd is None or now_ns - cmd.stamp_ns > self._deadman_timeout_ns:
            return Twist()
        return cmd.msg  # type: ignore[return-value]

    def _desired_vertical_speed_down(self) -> float:
        vehicle_odom = self._latest_vehicle_odom
        if vehicle_odom is None:
            return 0.0

        msg: VehicleOdometry = vehicle_odom.msg  # type: ignore[assignment]
        pd = float(msg.position[2])
        if not _is_finite(pd):
            return 0.0
        z_up = -pd
        v_up = self._alt_kp * (self._target_alt_m - z_up)
        v_up = _clamp(v_up, -self._max_descend_mps, self._max_climb_mps)
        return -v_up

    def _yaw_enu(self) -> Optional[float]:
        vehicle_odom = self._latest_vehicle_odom
        if vehicle_odom is None:
            return None
        msg: VehicleOdometry = vehicle_odom.msg  # type: ignore[assignment]
        return _yaw_enu_from_px4_q(list(msg.q))

    def _tick(self) -> None:
        now_ns = self._now_ns()

        offboard = OffboardControlMode()
        offboard.timestamp = self._px4_now_us()
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.thrust_and_torque = False
        offboard.direct_actuator = False
        self._offboard_pub.publish(offboard)

        cmd = self._latest_cmd_vel(now_ns)
        yaw_enu = self._yaw_enu() or 0.0

        v_fwd = float(cmd.linear.x)
        v_left = float(cmd.linear.y)
        yaw_rate_enu = float(cmd.angular.z)

        c = math.cos(yaw_enu)
        s = math.sin(yaw_enu)
        v_east = v_fwd * c - v_left * s
        v_north = v_fwd * s + v_left * c
        v_down = self._desired_vertical_speed_down()

        traj = TrajectorySetpoint()
        traj.timestamp = self._px4_now_us()
        traj.position = [math.nan, math.nan, math.nan]
        traj.velocity = [float(v_north), float(v_east), float(v_down)]
        traj.acceleration = [math.nan, math.nan, math.nan]
        traj.jerk = [math.nan, math.nan, math.nan]
        traj.yaw = math.nan
        traj.yawspeed = -float(yaw_rate_enu)
        self._traj_pub.publish(traj)

        self._setpoint_count += 1
        self._maybe_send_offboard_and_arm(now_ns)


def main() -> None:
    rclpy.init()
    node = TwistToPX4Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['TwistToPX4Offboard', 'main']
