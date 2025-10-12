"""Core autopilot publisher used by both entry points and scripts."""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AutoPilot(Node):
    """Publish a steady Twist command."""

    def __init__(self) -> None:
        super().__init__('auto_pilot')
        topic = os.environ.get('AP_TOPIC', '/cmd_vel')
        qos = int(os.environ.get('AP_QOS_DEPTH', '10'))
        self.pub = self.create_publisher(Twist, topic, qos)

        lin_x = float(os.environ.get('AP_LIN_X', '0.3'))
        lin_y = float(os.environ.get('AP_LIN_Y', '0.0'))
        lin_z = float(os.environ.get('AP_LIN_Z', '0.1'))
        ang_z = float(os.environ.get('AP_ANG_Z', '0.3'))

        self.alt_target = float(os.environ.get('AP_ALT_TARGET', '1.5'))
        self.alt_kp = float(os.environ.get('AP_ALT_KP', '0.8'))
        self.alt_deadband = float(os.environ.get('AP_ALT_DEADBAND', '0.05'))
        self.max_vert_speed = float(os.environ.get('AP_LIN_Z_MAX', str(abs(lin_z) or 0.2)))
        self.current_alt: float | None = None

        self.msg = Twist()
        self.msg.linear.x = lin_x
        self.msg.linear.y = lin_y
        self.msg.linear.z = 0.0
        self.msg.angular.z = ang_z

        hz = float(os.environ.get('AP_RATE', '10.0'))
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info(
            f'Publishing Twist on {topic} @ {hz} Hz: '
            f'lin=({lin_x},{lin_y},*) target_z={self.alt_target} ang.z={ang_z}'
        )

    def _tick(self) -> None:
        self.msg.linear.z = self._altitude_command()
        self.pub.publish(self.msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_alt = msg.pose.pose.position.z

    def _altitude_command(self) -> float:
        if self.current_alt is None:
            # Before odom arrives, climb gently.
            return min(self.max_vert_speed, 0.2)

        error = self.alt_target - self.current_alt
        if abs(error) <= self.alt_deadband:
            return 0.0

        cmd = self.alt_kp * error
        return max(-self.max_vert_speed, min(self.max_vert_speed, cmd))


def main() -> None:
    rclpy.init()
    node = AutoPilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['AutoPilot', 'main']
