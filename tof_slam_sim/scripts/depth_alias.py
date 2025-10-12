#!/usr/bin/env python3
"""Republish Gazebo depth topics to simple /tof_<sensor>/depth names."""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class DepthAlias(Node):
    """Bridge Gazebo depth topics onto concise ROS names."""

    def __init__(self) -> None:
        super().__init__('depth_alias')

        # Parameters mirror the launch file defaults so the alias stays in sync.
        self.world = self.declare_parameter('world', 'playfield').value
        self.model = self.declare_parameter('model', 'robot').value
        self.stream = self.declare_parameter('stream', 'depth_image').value

        self.sensor_names = [
            'front', 'front_right', 'right', 'back_right',
            'back', 'back_left', 'left', 'front_left',
        ]

        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._alias_publishers: Dict[str, Image] = {}
        for name in self.sensor_names:
            ros_topic = f'/tof_{name}/depth'
            gz_topic = (
                f'/world/{self.world}/model/{self.model}/model/tof_{name}'
                f'/link/sensor/depth_camera/{self.stream}'
            )

            self._alias_publishers[name] = self.create_publisher(Image, ros_topic, 10)
            self.create_subscription(
                Image,
                gz_topic,
                lambda msg, sensor=name: self._relay(sensor, msg),
                depth_qos,
            )
            self.get_logger().info(
                f'Forwarding {gz_topic} -> {ros_topic}',
            )

    def _relay(self, sensor: str, msg: Image) -> None:
        # Use a shallow copy to avoid mutating the original message that other
        # subscribers (if any) might read.
        alias = Image()
        alias.header = msg.header
        alias.height = msg.height
        alias.width = msg.width
        alias.encoding = msg.encoding
        alias.is_bigendian = msg.is_bigendian
        alias.step = msg.step
        alias.data = bytes(msg.data)
        alias.header.frame_id = f'tof_{sensor}'
        self._alias_publishers[sensor].publish(alias)


def main() -> None:
    rclpy.init()
    node = DepthAlias()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
