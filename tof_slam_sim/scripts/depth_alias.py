#!/usr/bin/env python3
"""Republish Gazebo depth topics to simple /tof_<sensor>/depth names."""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


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
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._alias_publishers: Dict[str, Publisher] = {}
        self._alias_info_publishers: Dict[str, Publisher] = {}
        for name in self.sensor_names:
            ros_topic = f'/tof_{name}/depth'
            ros_info_topic = f'/tof_{name}/camera_info'
            gz_topic = (
                f'/world/{self.world}/model/{self.model}/model/tof_{name}'
                f'/link/sensor/depth_camera/{self.stream}'
            )
            gz_info_topic = gz_topic.rsplit('/', 1)[0] + '/camera_info'

            self._alias_publishers[name] = self.create_publisher(Image, ros_topic, depth_qos)
            self._alias_info_publishers[name] = self.create_publisher(CameraInfo, ros_info_topic, camera_info_qos)
            self.create_subscription(
                Image,
                gz_topic,
                lambda msg, sensor=name: self._relay(sensor, msg),
                depth_qos,
            )
            self.create_subscription(
                CameraInfo,
                gz_info_topic,
                lambda msg, sensor=name: self._relay_info(sensor, msg),
                camera_info_qos,
            )
            self.get_logger().info(
                f'Forwarding {gz_topic} -> {ros_topic}',
            )
            self.get_logger().info(
                f'Forwarding {gz_info_topic} -> {ros_info_topic}',
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

    def _relay_info(self, sensor: str, msg: CameraInfo) -> None:
        alias = CameraInfo()
        alias.header = msg.header
        alias.height = msg.height
        alias.width = msg.width
        alias.distortion_model = msg.distortion_model
        alias.d = list(msg.d)
        alias.k = list(msg.k)
        alias.r = list(msg.r)
        alias.p = list(msg.p)
        alias.binning_x = msg.binning_x
        alias.binning_y = msg.binning_y
        alias.roi = msg.roi
        alias.header.frame_id = f'tof_{sensor}'
        self._alias_info_publishers[sensor].publish(alias)


def main() -> None:
    rclpy.init()
    node = DepthAlias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
