#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
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


def _quat_xyzw_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_enu_from_px4_q(q_wxyz: list[float]) -> Optional[float]:
    if len(q_wxyz) != 4:
        return None
    w, x, y, z = (float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))
    if not all(_is_finite(v) for v in (w, x, y, z)):
        return None

    # PX4: q is FRD(body) -> NED(world). We want FLU(body) -> ENU(world).
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
class _LastGood:
    x: float
    y: float
    yaw: Optional[float]


class PX4VehicleOdometryToOdom(Node):
    def __init__(self) -> None:
        super().__init__('px4_vehicle_odometry_to_odom')

        self.declare_parameter('vehicle_odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'robot/odom')
        self.declare_parameter('child_frame_id', 'robot/base_footprint')
        self.declare_parameter('yaw_only', True)
        self.declare_parameter('use_message_z', False)
        self.declare_parameter('z_override', 0.0)
        self.declare_parameter('drop_invalid_position', False)
        self.declare_parameter('hold_last_position', True)
        self.declare_parameter('warn_period_sec', 5.0)

        self._vehicle_topic = (
            str(self.get_parameter('vehicle_odometry_topic').value).strip()
            or '/fmu/out/vehicle_odometry'
        )
        self._odom_topic = str(self.get_parameter('odom_topic').value).strip() or '/odom'
        self._frame_id = str(self.get_parameter('frame_id').value).strip() or 'robot/odom'
        self._child_frame_id = (
            str(self.get_parameter('child_frame_id').value).strip() or 'robot/base_footprint'
        )
        self._yaw_only = bool(self.get_parameter('yaw_only').value)
        self._use_message_z = bool(self.get_parameter('use_message_z').value)
        self._z_override = float(self.get_parameter('z_override').value)
        self._drop_invalid_position = bool(self.get_parameter('drop_invalid_position').value)
        self._hold_last_position = bool(self.get_parameter('hold_last_position').value)

        warn_period_sec = float(self.get_parameter('warn_period_sec').value)
        self._warn_period_ns = int(max(0.0, warn_period_sec) * 1e9)

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Publish as RELIABLE so both RELIABLE and BEST_EFFORT subscribers can match.
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Odometry, self._odom_topic, qos_pub)
        self.create_subscription(VehicleOdometry, self._vehicle_topic, self._on_vehicle_odom, qos_sub)

        self._warned_frame = False
        self._last_good: Optional[_LastGood] = None
        self._last_warn_ns: Optional[int] = None

        self.get_logger().info(
            f'PX4 vehicle_odometry -> Odometry: {self._vehicle_topic} -> {self._odom_topic} '
            f'({self._frame_id}->{self._child_frame_id}, yaw_only={self._yaw_only})'
        )

    def _now_stamp(self):
        return self.get_clock().now().to_msg()

    def _on_vehicle_odom(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED and not self._warned_frame:
            self._warned_frame = True
            self.get_logger().warn(
                f'Unexpected pose_frame={int(msg.pose_frame)}; expected NED. '
                'Continuing, but transforms may be wrong.'
            )

        pn = float(msg.position[0])
        pe = float(msg.position[1])
        pd = float(msg.position[2])
        pos_valid = all(_is_finite(v) for v in (pn, pe, pd))

        yaw_enu_msg = _yaw_enu_from_px4_q(list(msg.q))

        if not pos_valid and self._drop_invalid_position:
            return

        if pos_valid:
            x_enu = pe
            y_enu = pn
            z_enu = -pd
            if not self._use_message_z:
                z_enu = float(self._z_override)
        else:
            x_enu = 0.0
            y_enu = 0.0
            if self._hold_last_position and self._last_good is not None:
                x_enu = self._last_good.x
                y_enu = self._last_good.y

            z_enu = float(self._z_override)
            now_ns = self.get_clock().now().nanoseconds
            if self._warn_period_ns > 0:
                should_warn = (
                    self._last_warn_ns is None
                    or now_ns - self._last_warn_ns >= self._warn_period_ns
                )
            else:
                should_warn = self._last_warn_ns is None
            if should_warn:
                mode = 'hold_last' if (self._hold_last_position and self._last_good is not None) else 'zero'
                self.get_logger().warn(
                    'PX4 VehicleOdometry position is invalid (NaN/inf); publishing fallback /odom '
                    f'({mode}). Check EKF2 aiding (optical flow/GPS) and uXRCE topics.'
                )
                self._last_warn_ns = now_ns

        yaw_enu = yaw_enu_msg
        if yaw_enu is None and self._last_good is not None:
            yaw_enu = self._last_good.yaw

        odom = Odometry()
        odom.header.stamp = self._now_stamp()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id

        odom.pose.pose.position.x = float(x_enu)
        odom.pose.pose.position.y = float(y_enu)
        odom.pose.pose.position.z = float(z_enu)

        if yaw_enu is None:
            odom.pose.pose.orientation.w = 1.0
        elif self._yaw_only:
            qx, qy, qz, qw = _quat_xyzw_from_yaw(yaw_enu)
            odom.pose.pose.orientation.x = float(qx)
            odom.pose.pose.orientation.y = float(qy)
            odom.pose.pose.orientation.z = float(qz)
            odom.pose.pose.orientation.w = float(qw)
        else:
            # Full quaternion conversion is not currently implemented; keep yaw-only.
            qx, qy, qz, qw = _quat_xyzw_from_yaw(yaw_enu)
            odom.pose.pose.orientation.x = float(qx)
            odom.pose.pose.orientation.y = float(qy)
            odom.pose.pose.orientation.z = float(qz)
            odom.pose.pose.orientation.w = float(qw)

        vx_fwd = 0.0
        vy_left = 0.0
        wz = 0.0

        vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED and all(_is_finite(v) for v in vel):
            vn, ve, vd = vel
            vx_e = ve
            vy_n = vn
            if yaw_enu is not None:
                c = math.cos(yaw_enu)
                s = math.sin(yaw_enu)
                vx_fwd = vx_e * c + vy_n * s
                vy_left = -vx_e * s + vy_n * c
        elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD and all(
            _is_finite(v) for v in vel
        ):
            vf, vr, vd = vel
            vx_fwd = vf
            vy_left = -vr

        ang = [
            float(msg.angular_velocity[0]),
            float(msg.angular_velocity[1]),
            float(msg.angular_velocity[2]),
        ]
        if all(_is_finite(v) for v in ang):
            wz = -ang[2]
            wz = _clamp(wz, -10.0, 10.0)

        odom.twist.twist.linear.x = float(vx_fwd)
        odom.twist.twist.linear.y = float(vy_left)
        odom.twist.twist.angular.z = float(wz)

        if pos_valid:
            last_yaw = yaw_enu_msg if yaw_enu_msg is not None else (self._last_good.yaw if self._last_good else None)
            self._last_good = _LastGood(x=float(x_enu), y=float(y_enu), yaw=last_yaw)

        self._pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = PX4VehicleOdometryToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['PX4VehicleOdometryToOdom', 'main']
