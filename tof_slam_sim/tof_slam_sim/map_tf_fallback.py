#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MapTFFallback(Node):
    """Broadcast a one-shot identity map->odom transform at startup."""

    def __init__(self) -> None:
        super().__init__('map_tf_fallback')
        self.declare_parameter('parent_frame', 'robot/map')
        self.declare_parameter('child_frame', 'robot/odom')
        self.parent = (
            self.get_parameter('parent_frame').get_parameter_value().string_value
        )
        self.child = (
            self.get_parameter('child_frame').get_parameter_value().string_value
        )

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self._tick)
        self._sent = False
        self.get_logger().info(
            f'Publishing single identity TF {self.parent}->{self.child} during startup.'
        )

    def _tick(self) -> None:
        if self._sent:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self._sent = True
        self.destroy_timer(self.timer)
        self.timer = None
        self.get_logger().info(
            f'Fallback TF {self.parent}->{self.child} broadcast once; awaiting live publisher.'
        )


def main() -> None:
    rclpy.init()
    node = MapTFFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, 'timer', None) is not None:
            node.destroy_timer(node.timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['MapTFFallback', 'main']
