#!/usr/bin/env python3
"""Core autopilot publisher (stable/minimal)."""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class AutoPilot(Node):
    """Publish a steady Twist command (planar by default)."""

    def __init__(self) -> None:
        super().__init__('auto_pilot')

        # Topic & QoS
        topic = os.environ.get('AP_TOPIC', '/cmd_vel')
        self.pub = self.create_publisher(Twist, topic, qos_reliable(10))

        # Kinematics
        lin_x = float(os.environ.get('AP_LIN_X', '0.3'))
        lin_y = float(os.environ.get('AP_LIN_Y', '0.0'))
        ang_z = float(os.environ.get('AP_ANG_Z', '0.3'))
        self.msg = Twist()
        self.msg.linear.x = lin_x
        self.msg.linear.y = lin_y
        self.msg.linear.z = 0.0
        self.msg.angular.z = ang_z

        # Optional altitude control (off by default to avoid rmw issues)
        self.alt_enable = os.environ.get('AP_ALT_ENABLE', '0').lower() in ('1', 'true', 'yes')
        self.current_alt: float | None = None
        self.alt_target = float(os.environ.get('AP_ALT_TARGET', '1.5'))
        self.alt_kp = float(os.environ.get('AP_ALT_KP', '0.8'))
        self.alt_deadband = float(os.environ.get('AP_ALT_DEADBAND', '0.05'))
        self.max_vert_speed = float(os.environ.get('AP_LIN_Z_MAX', '0.2'))
        if self.alt_enable:
            odom_topic = os.environ.get('AP_ODOM', '/odom')
            self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_reliable(10))

        # Rate
        hz = float(os.environ.get('AP_RATE', '10.0'))
        period = 1.0 / hz if hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Publishing Twist on {topic} @ {hz:.1f} Hz: '
            f'lin=({lin_x},{lin_y},0.0) ang.z={ang_z} alt_ctrl={self.alt_enable}'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        # keep minimal to avoid NaN math surprises
        self.current_alt = float(msg.pose.pose.position.z)

    def _altitude_command(self) -> float:
        if not self.alt_enable:
            return 0.0
        if self.current_alt is None:
            # gentle initial climb
            return min(self.max_vert_speed, 0.2)
        error = self.alt_target - self.current_alt
        if abs(error) <= self.alt_deadband:
            return 0.0
        cmd = self.alt_kp * error
        return max(-self.max_vert_speed, min(self.max_vert_speed, cmd))

    def _tick(self) -> None:
        # keep all math here (timer context)
        self.msg.linear.z = self._altitude_command()
        self.pub.publish(self.msg)


def main() -> None:
    # faulthandler via env is set in launch; keep normal init here
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
