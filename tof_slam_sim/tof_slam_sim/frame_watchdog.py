"""Monitor TF transforms and report when required links are missing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


@dataclass
class TransformStatus:
    parent: str
    child: str
    last_seen: Time | None = None
    warned_missing: bool = False


class FrameWatchdog(Node):
    """Periodically ensure required TF transforms are available."""

    def __init__(self) -> None:
        super().__init__('frame_watchdog')
        default_pairs = [
            'map->odom',
            'odom->base_link',
            'map->base_link',
        ]
        pairs = self.declare_parameter('transforms', default_pairs).value
        interval = float(self.declare_parameter('check_period_sec', 1.0).value)
        stale_sec = float(self.declare_parameter('stale_after_sec', 3.0).value)
        self.stale_limit = Duration(seconds=stale_sec)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._transforms: Dict[Tuple[str, str], TransformStatus] = {}
        for raw in pairs:
            if '->' not in raw:
                self.get_logger().warn(f'Ignoring malformed transform spec "{raw}" (use parent->child).')
                continue
            parent, child = [part.strip() for part in raw.split('->', 1)]
            key = (parent, child)
            self._transforms[key] = TransformStatus(parent, child)

        self.timer = self.create_timer(interval, self._check)
        listing = ', '.join(f'{p}->{c}' for p, c in self._transforms)
        self.get_logger().info(f'Monitoring transforms: {listing}')

    def _check(self) -> None:
        now = self.get_clock().now()
        for status in self._transforms.values():
            try:
                tf = self.tf_buffer.lookup_transform(
                    status.parent,
                    status.child,
                    rclpy.time.Time())
            except TransformException as exc:
                if not status.warned_missing:
                    self.get_logger().warning(
                        f'Transform {status.parent} -> {status.child} missing ({exc.__class__.__name__})'
                    )
                    status.warned_missing = True
                continue

            status.last_seen = Time.from_msg(tf.header.stamp)
            if status.warned_missing:
                self.get_logger().info(
                    'Transform %s -> %s is now available', status.parent, status.child
                )
            status.warned_missing = False

            if now - status.last_seen > self.stale_limit:
                elapsed = (now - status.last_seen).nanoseconds * 1e-9
                self.get_logger().warning(
                    f'Transform {status.parent} -> {status.child} is stale (last {elapsed:.2f}s ago)'
                )


def main() -> None:
    rclpy.init()
    node = FrameWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['FrameWatchdog', 'main']
