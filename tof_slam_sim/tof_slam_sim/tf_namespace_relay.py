#!/usr/bin/env python3
from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _strip_leading_slash(frame: str) -> str:
    return frame[1:] if frame.startswith('/') else frame


class TFNamespaceRelay(Node):
    """Republish filtered TF into a robot namespace.

    Nav2 bringup remaps `/tf` -> `tf` and `/tf_static` -> `tf_static` so namespaced
    stacks listen on `/<ns>/tf`. This relay copies the global TF streams into the
    namespace while filtering to transforms relevant to a specific robot prefix.
    """

    def __init__(self) -> None:
        super().__init__('tf_namespace_relay')

        self.declare_parameter('robot_prefix', '')
        self.declare_parameter('in_tf_topic', '/tf')
        self.declare_parameter('in_tf_static_topic', '/tf_static')
        self.declare_parameter('out_tf_topic', 'tf')
        self.declare_parameter('out_tf_static_topic', 'tf_static')

        self._prefix = str(self.get_parameter('robot_prefix').value).strip()
        self._in_tf = str(self.get_parameter('in_tf_topic').value).strip() or '/tf'
        self._in_tf_static = str(self.get_parameter('in_tf_static_topic').value).strip() or '/tf_static'
        self._out_tf = str(self.get_parameter('out_tf_topic').value).strip() or 'tf'
        self._out_tf_static = str(self.get_parameter('out_tf_static_topic').value).strip() or 'tf_static'

        pub_tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_tf = self.create_publisher(TFMessage, self._out_tf, pub_tf_qos)
        self._pub_tf_static = self.create_publisher(TFMessage, self._out_tf_static, pub_static_qos)

        sub_tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            durability=DurabilityPolicy.VOLATILE,
        )
        sub_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, self._in_tf, self._on_tf, sub_tf_qos)
        self.create_subscription(TFMessage, self._in_tf_static, self._on_tf_static, sub_static_qos)

        self.get_logger().info(
            f'Relaying TF into namespace topics "{self._out_tf}" + "{self._out_tf_static}" '
            f'(filter prefix="{self._prefix or "<none>"}").'
        )

    def _filter(self, msg: TFMessage) -> List:
        if not self._prefix:
            return list(msg.transforms)

        prefix = f'{self._prefix}/'
        out = []
        for t in msg.transforms:
            parent = _strip_leading_slash(str(t.header.frame_id))
            child = _strip_leading_slash(str(t.child_frame_id))
            if parent.startswith(prefix) or child.startswith(prefix):
                out.append(t)
        return out

    def _on_tf(self, msg: TFMessage) -> None:
        transforms = self._filter(msg)
        if not transforms:
            return
        out = TFMessage()
        out.transforms = transforms
        self._pub_tf.publish(out)

    def _on_tf_static(self, msg: TFMessage) -> None:
        transforms = self._filter(msg)
        if not transforms:
            return
        out = TFMessage()
        out.transforms = transforms
        self._pub_tf_static.publish(out)


def main() -> None:
    rclpy.init()
    node = TFNamespaceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['TFNamespaceRelay', 'main']

