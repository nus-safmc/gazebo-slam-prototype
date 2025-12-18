#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


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


def _lerp_angle(prev: float, new: float, alpha: float) -> float:
    delta = _wrap_angle(new - prev)
    return _wrap_angle(prev + alpha * delta)


@dataclass
class _State:
    x: float
    y: float
    yaw: float


class OdomTFPublisher(Node):
    def __init__(self) -> None:
        super().__init__('odom_tf_publisher')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('child_frame', 'robot/base_footprint')
        self.declare_parameter('yaw_only', True)
        self.declare_parameter('use_message_z', False)
        self.declare_parameter('z_override', 0.0)
        self.declare_parameter('smoothing_alpha', 1.0)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._parent_frame = str(self.get_parameter('parent_frame').value).strip()
        self._child_frame = str(self.get_parameter('child_frame').value).strip()
        self._yaw_only = bool(self.get_parameter('yaw_only').value)
        self._use_message_z = bool(self.get_parameter('use_message_z').value)
        self._z_override = float(self.get_parameter('z_override').value)

        alpha = float(self.get_parameter('smoothing_alpha').value)
        self._alpha = min(1.0, max(0.0, alpha))

        self._tf = TransformBroadcaster(self)
        self._state: Optional[_State] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, qos)

        self.get_logger().info(
            f'Publishing TF from odom "{self._odom_topic}" as '
            f'{self._parent_frame or "<odom.header.frame_id>"}->{self._child_frame} '
            f'(yaw_only={self._yaw_only}, alpha={self._alpha:.2f}).'
        )

    def _on_odom(self, msg: Odometry) -> None:
        parent = self._parent_frame or str(msg.header.frame_id).strip() or 'odom'
        child = self._child_frame or str(msg.child_frame_id).strip() or 'base_footprint'

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w) if self._yaw_only else None

        x = float(p.x)
        y = float(p.y)
        if yaw is None:
            # Full quaternion path: keep x/y and use incoming orientation.
            new_state = _State(x=x, y=y, yaw=0.0)
        else:
            new_state = _State(x=x, y=y, yaw=float(yaw))

        if self._state is None or self._alpha >= 0.999:
            state = new_state
        else:
            prev = self._state
            state = _State(
                x=prev.x + self._alpha * (new_state.x - prev.x),
                y=prev.y + self._alpha * (new_state.y - prev.y),
                yaw=_lerp_angle(prev.yaw, new_state.yaw, self._alpha),
            )
        self._state = state

        t = TransformStamped()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            t.header.stamp = self.get_clock().now().to_msg()
        else:
            t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = float(p.z) if self._use_message_z else self._z_override

        if self._yaw_only:
            qx, qy, qz, qw = _quat_from_yaw(state.yaw)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
        else:
            t.transform.rotation = msg.pose.pose.orientation

        self._tf.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['OdomTFPublisher', 'main']
