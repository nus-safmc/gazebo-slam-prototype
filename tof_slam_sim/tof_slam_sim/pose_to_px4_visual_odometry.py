#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _q_wxyz_from_yaw_enu(yaw_enu: float) -> tuple[float, float, float, float]:
    # Build PX4-style quaternion (wxyz) such that the conversion used in
    # `px4_vehicle_odometry_to_odom` yields `yaw_enu`.
    yaw_ned = math.pi / 2.0 - float(yaw_enu)
    half = 0.5 * yaw_ned
    return math.cos(half), 0.0, 0.0, math.sin(half)


@dataclass
class _State:
    stamp_ns: int
    x_e: float
    y_n: float
    z_u: float
    yaw_enu: float


class PoseToPX4VisualOdometry(Node):
    """Publish Gazebo pose as PX4 vehicle_visual_odometry (uXRCE-DDS).

    This enables EKF2 external-vision aiding when the simulated airframe has no GPS/mag.
    """

    def __init__(self) -> None:
        super().__init__('pose_to_px4_visual_odometry')

        self.declare_parameter('pose_topic', '/model/x500_small_tof_0/pose')
        self.declare_parameter('px4_time_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('px4_visual_odometry_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.declare_parameter('position_stddev_m', 0.05)
        self.declare_parameter('velocity_stddev_mps', 0.10)
        self.declare_parameter('yaw_stddev_rad', 0.10)

        self._pose_topic = str(self.get_parameter('pose_topic').value).strip()
        self._px4_time_topic = str(self.get_parameter('px4_time_topic').value).strip()
        self._px4_vo_topic = str(self.get_parameter('px4_visual_odometry_topic').value).strip()

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._min_pub_period_ns = 0
        if publish_rate_hz > 0.0:
            self._min_pub_period_ns = int(1e9 / max(0.1, publish_rate_hz))
        self._last_pub_ns = 0

        self._pos_var = float(self.get_parameter('position_stddev_m').value) ** 2
        self._vel_var = float(self.get_parameter('velocity_stddev_mps').value) ** 2
        self._yaw_var = float(self.get_parameter('yaw_stddev_rad').value) ** 2

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._px4_ts_us: Optional[int] = None
        self.create_subscription(VehicleOdometry, self._px4_time_topic, self._on_px4_time, qos_px4)

        self._pub = self.create_publisher(VehicleOdometry, self._px4_vo_topic, qos_px4)
        self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, qos_pose)

        self._prev: Optional[_State] = None

        self.get_logger().info(
            'Publishing PX4 vehicle_visual_odometry from Gazebo pose: '
            f'{self._pose_topic} -> {self._px4_vo_topic} '
            f'(time base: {self._px4_time_topic})'
        )

    def _on_px4_time(self, msg: VehicleOdometry) -> None:
        self._px4_ts_us = int(msg.timestamp)

    def _pose_stamp_ns(self, msg: PoseStamped) -> int:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return int(self.get_clock().now().nanoseconds)
        return int(stamp.sec * 1_000_000_000 + stamp.nanosec)

    def _on_pose(self, msg: PoseStamped) -> None:
        if self._px4_ts_us is None:
            return

        stamp_ns = self._pose_stamp_ns(msg)
        if self._min_pub_period_ns > 0 and (stamp_ns - self._last_pub_ns) < self._min_pub_period_ns:
            return
        self._last_pub_ns = stamp_ns

        p = msg.pose.position
        q = msg.pose.orientation

        x_e = float(p.x)
        y_n = float(p.y)
        z_u = float(p.z)

        yaw_enu = float(_yaw_from_quat(q.x, q.y, q.z, q.w))

        ve = vn = vu = 0.0
        yaw_rate_enu = 0.0

        if self._prev is not None:
            dt = (stamp_ns - self._prev.stamp_ns) * 1e-9
            if dt > 1e-4:
                ve = (x_e - self._prev.x_e) / dt
                vn = (y_n - self._prev.y_n) / dt
                vu = (z_u - self._prev.z_u) / dt
                yaw_rate_enu = _wrap_angle(yaw_enu - self._prev.yaw_enu) / dt

        self._prev = _State(stamp_ns=stamp_ns, x_e=x_e, y_n=y_n, z_u=z_u, yaw_enu=yaw_enu)

        # ENU -> NED
        pn = y_n
        pe = x_e
        pd = -z_u

        vn_ned = vn
        ve_ned = ve
        vd_ned = -vu

        qw, qx, qy, qz = _q_wxyz_from_yaw_enu(yaw_enu)

        vo = VehicleOdometry()
        vo.timestamp = int(self._px4_ts_us)
        vo.timestamp_sample = int(self._px4_ts_us)

        vo.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vo.position = [float(pn), float(pe), float(pd)]
        vo.q = [float(qw), float(qx), float(qy), float(qz)]

        vo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        vo.velocity = [float(vn_ned), float(ve_ned), float(vd_ned)]

        # Body angular velocity (FRD). For yaw-only, map ENU yaw-rate to FRD z-rate.
        vo.angular_velocity = [math.nan, math.nan, float(-yaw_rate_enu)]

        vo.position_variance = [self._pos_var, self._pos_var, self._pos_var]
        vo.velocity_variance = [self._vel_var, self._vel_var, self._vel_var]
        vo.orientation_variance = [math.nan, math.nan, self._yaw_var]

        vo.reset_counter = 0
        vo.quality = 100

        self._pub.publish(vo)


def main() -> None:
    rclpy.init()
    node = PoseToPX4VisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['PoseToPX4VisualOdometry', 'main']

