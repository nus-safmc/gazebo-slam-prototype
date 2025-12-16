#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class CmdVelRelay(Node):
    """Relay Nav2's `cmd_vel_nav` to the robot `cmd_vel` topic."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_relay')

        self.declare_parameter('input_topic', 'cmd_vel_nav')
        self.declare_parameter('output_topic', 'cmd_vel')

        self._in = str(self.get_parameter('input_topic').value).strip() or 'cmd_vel_nav'
        self._out = str(self.get_parameter('output_topic').value).strip() or 'cmd_vel'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Twist, self._out, qos)
        self.create_subscription(Twist, self._in, self._cb, qos)

        self.get_logger().info(f'Relaying Twist: {self._in} -> {self._out}')

    def _cb(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['CmdVelRelay', 'main']

