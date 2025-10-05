"""Core autopilot publisher used by both entry points and scripts."""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AutoPilot(Node):
    """Publish a steady Twist command."""

    def __init__(self) -> None:
        super().__init__('auto_pilot')
        topic = os.environ.get('AP_TOPIC', '/cmd_vel')
        qos = int(os.environ.get('AP_QOS_DEPTH', '10'))
        self.pub = self.create_publisher(Twist, topic, qos)

        lin_x = float(os.environ.get('AP_LIN_X', '0.8'))
        lin_y = float(os.environ.get('AP_LIN_Y', '0.0'))
        lin_z = float(os.environ.get('AP_LIN_Z', '0.6'))
        ang_z = float(os.environ.get('AP_ANG_Z', '0.2'))

        self.msg = Twist()
        self.msg.linear.x = lin_x
        self.msg.linear.y = lin_y
        self.msg.linear.z = lin_z
        self.msg.angular.z = ang_z

        hz = float(os.environ.get('AP_RATE', '10.0'))
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'Publishing Twist on {topic} @ {hz} Hz: '
            f'lin=({lin_x},{lin_y},{lin_z}) ang.z={ang_z}'
        )

    def _tick(self) -> None:
        self.pub.publish(self.msg)


def main() -> None:
    rclpy.init()
    node = AutoPilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['AutoPilot', 'main']
