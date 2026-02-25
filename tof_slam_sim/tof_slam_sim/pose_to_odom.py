#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class _State:
    stamp_ns: int
    x: float
    y: float
    yaw: float


class PoseToOdom(Node):
    def __init__(self) -> None:
        super().__init__('pose_to_odom')

        self.declare_parameter('pose_topic', '/model/x500_small_tof_0/pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'robot/odom')
        self.declare_parameter('child_frame_id', 'robot/base_footprint')
        self.declare_parameter('yaw_only', True)
        self.declare_parameter('use_message_z', False)
        self.declare_parameter('z_override', 0.0)

        self._pose_topic = str(self.get_parameter('pose_topic').value).strip() or '/model/x500_small_tof_0/pose'
        self._odom_topic = str(self.get_parameter('odom_topic').value).strip() or '/odom'
        self._frame_id = str(self.get_parameter('frame_id').value).strip() or 'robot/odom'
        self._child_frame_id = str(self.get_parameter('child_frame_id').value).strip() or 'robot/base_footprint'
        self._yaw_only = bool(self.get_parameter('yaw_only').value)
        self._use_message_z = bool(self.get_parameter('use_message_z').value)
        self._z_override = float(self.get_parameter('z_override').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Odometry, self._odom_topic, qos)
        self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, qos)

        self._prev: Optional[_State] = None

        self.get_logger().info(
            f'Publishing Odometry: {self._pose_topic} -> {self._odom_topic} '
            f'({self._frame_id}->{self._child_frame_id}, yaw_only={self._yaw_only})'
        )

    def _stamp_ns(self, msg: PoseStamped) -> int:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return int(self.get_clock().now().nanoseconds)
        return int(stamp.sec * 1_000_000_000 + stamp.nanosec)

    def _on_pose(self, msg: PoseStamped) -> None:
        now_ns = self._stamp_ns(msg)

        p = msg.pose.position
        q = msg.pose.orientation
        x = float(p.x)
        y = float(p.y)
        yaw = float(_yaw_from_quat(q.x, q.y, q.z, q.w))

        vx_fwd = 0.0
        vy_left = 0.0
        wz = 0.0
        if self._prev is not None:
            dt = (now_ns - self._prev.stamp_ns) * 1e-9
            if dt > 1e-4:
                dx = x - self._prev.x
                dy = y - self._prev.y
                dyaw = _wrap_angle(yaw - self._prev.yaw)

                vx_east = dx / dt
                vy_north = dy / dt
                wz = dyaw / dt

                c = math.cos(yaw)
                s = math.sin(yaw)
                vx_fwd = vx_east * c + vy_north * s
                vy_left = -vx_east * s + vy_north * c
        self._prev = _State(stamp_ns=now_ns, x=x, y=y, yaw=yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = float(p.z) if self._use_message_z else self._z_override

        if self._yaw_only:
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
        else:
            odom.pose.pose.orientation = msg.pose.orientation

        odom.twist.twist.linear.x = float(vx_fwd)
        odom.twist.twist.linear.y = float(vy_left)
        odom.twist.twist.angular.z = float(wz)

        self._pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['PoseToOdom', 'main']

