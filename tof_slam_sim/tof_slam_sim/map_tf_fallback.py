#!/usr/bin/env python3
"""
Publish an identity map->odom transform ONLY UNTIL a real map->odom appears,
then stop publishing to avoid TF conflicts.

- Stamps with the node clock (respects /use_sim_time if set in launch/YAML).
- Checks TF buffer each tick; once a real map->odom exists, disables itself.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class MapTFFallback(Node):
    def __init__(self) -> None:
        super().__init__('map_tf_fallback')

        # Parameters (do NOT declare use_sim_time; let launch/YAML own it)
        self.declare_parameter('parent_frame', 'robot/map')
        self.declare_parameter('child_frame', 'robot/odom')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('one_shot', True)  # publish only until real TF appears

        self.parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child = self.get_parameter('child_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.one_shot = self.get_parameter('one_shot').get_parameter_value().bool_value

        # TF2 buffer/listener to detect when the REAL map->odom exists
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.br = TransformBroadcaster(self)

        self._active = True
        self._timer = self.create_timer(max(1e-3, 1.0 / rate), self._on_timer)
        self.get_logger().info(
            f'Waiting for real TF {self.parent} -> {self.child}; '
            f'publishing identity fallback at {rate:.1f} Hz until then.'
        )

    def _real_tf_available(self) -> bool:
        # Immediate check (no timeout) whether TF exists at "now"
        try:
            return self.buffer.can_transform(
                self.parent, self.child, Time(), timeout=Duration(seconds=0.0)
            )
        except Exception:
            return False

    def _publish_identity_once(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        # Identity transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

    def _on_timer(self) -> None:
        if not self._active:
            return

        if self._real_tf_available():
            self.get_logger().info(f'Detected REAL {self.parent}->{self.child}; stopping fallback publisher.')
            # Stop publishing to avoid duplicate map->odom authorities
            if self.one_shot:
                self._active = False
                self._timer.cancel()
            return

        # No real TF yet — publish identity
        self._publish_identity_once()


def main() -> None:
    rclpy.init()
    node = MapTFFallback()
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
