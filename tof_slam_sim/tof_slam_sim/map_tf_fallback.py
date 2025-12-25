#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class MapTFFallback(Node):
    """Publish an identity map->odom transform for early consumers.

    In Nav2+SLAM stacks, /map->/odom is usually published by a localization / SLAM node.
    If those nodes start late (or are waiting on other TF), message_filters can fill up
    and drop sensor messages because the TF tree is incomplete.

    Default behavior publishes a latched /tf_static identity transform so late-joiners
    still receive it. Downstream nodes can safely override it by publishing a dynamic
    map->odom transform later.
    """

    def __init__(self) -> None:
        super().__init__('map_tf_fallback')
        self.declare_parameter('parent_frame', 'robot/map')
        self.declare_parameter('child_frame', 'robot/odom')
        self.declare_parameter('use_static', True)
        self.declare_parameter('rate_hz', 10.0)

        self.parent = (
            self.get_parameter('parent_frame').get_parameter_value().string_value
        )
        self.child = (
            self.get_parameter('child_frame').get_parameter_value().string_value
        )
        self._use_static = bool(self.get_parameter('use_static').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)

        self._sent_once = False

        if self._use_static:
            self._static_br = StaticTransformBroadcaster(self)
            self._dynamic_br = None
            self._timer = None
            self._send_static_once()
        else:
            self._static_br = None
            self._dynamic_br = TransformBroadcaster(self)
            period = 0.1
            if self._rate_hz > 0.0:
                period = 1.0 / max(0.1, self._rate_hz)
            self._timer = self.create_timer(period, self._send_dynamic)
            self.get_logger().info(
                f'Publishing identity TF {self.parent}->{self.child} at {self._rate_hz:.1f} Hz '
                '(will be overridden by live publisher).'
            )

    def _make_identity(self) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.rotation.w = 1.0
        return t

    def _send_static_once(self) -> None:
        if self._sent_once:
            return
        t = self._make_identity()
        assert self._static_br is not None
        self._static_br.sendTransform(t)
        self._sent_once = True
        self.get_logger().info(
            f'Published latched /tf_static identity {self.parent}->{self.child}; awaiting live publisher.'
        )

    def _send_dynamic(self) -> None:
        if self._dynamic_br is None:
            return
        self._dynamic_br.sendTransform(self._make_identity())


def main() -> None:
    rclpy.init()
    node = MapTFFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        timer = getattr(node, '_timer', None)
        if timer is not None:
            node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['MapTFFallback', 'main']
