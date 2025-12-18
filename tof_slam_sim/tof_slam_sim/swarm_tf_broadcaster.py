#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _strip_leading_slash(frame: str) -> str:
    return frame[1:] if frame.startswith('/') else frame


def _filter_transforms(transforms: Iterable[TransformStamped], *, prefix: str) -> list[TransformStamped]:
    if not prefix:
        return list(transforms)

    want = f'{prefix}/'
    out: list[TransformStamped] = []
    for t in transforms:
        parent = _strip_leading_slash(str(t.header.frame_id))
        child = _strip_leading_slash(str(t.child_frame_id))
        if parent.startswith(want) or child.startswith(want):
            out.append(t)
    return out


class SwarmTFBroadcaster(Node):
    """Publish required TF for multi-robot Nav2 stacks with fewer DDS participants.

    This replaces many per-robot `static_transform_publisher`, `odom_tf_publisher`,
    and `tf_namespace_relay` processes with a single node.
    """

    def __init__(self) -> None:
        super().__init__('swarm_tf_broadcaster')

        self.declare_parameter('robots', ['robot', 'robot2', 'robot3', 'robot4'])
        self.declare_parameter('map_frame', 'robot/map')
        self.declare_parameter('publish_base_link_alias', True)

        self._robots = [str(r).strip() for r in self.get_parameter('robots').value if str(r).strip()]
        if not self._robots:
            self._robots = ['robot']

        self._map_frame = str(self.get_parameter('map_frame').value).strip() or 'robot/map'
        self._publish_alias = bool(self.get_parameter('publish_base_link_alias').value)

        qos_tf = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_tf_static = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_pub = self.create_publisher(TFMessage, '/tf', qos_tf)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', qos_tf_static)

        self._ns_tf_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._ns_tf_static_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for r in self._robots:
            if r == 'robot':
                continue
            self._ns_tf_pubs[r] = self.create_publisher(TFMessage, f'/{r}/tf', qos_tf)
            self._ns_tf_static_pubs[r] = self.create_publisher(
                TFMessage, f'/{r}/tf_static', qos_tf_static
            )

        static_transforms = self._build_static_transforms()
        self._publish_static(static_transforms)

        # Use BEST_EFFORT to be compatible with both reliable + best-effort odom publishers.
        # Some bridges/simulators publish Odometry with sensor-data QoS.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odom_subs = []
        for r in self._robots:
            odom_topic = '/odom' if r == 'robot' else f'/{r}/odom'
            self._odom_subs.append(
                self.create_subscription(Odometry, odom_topic, self._mk_odom_cb(r), odom_qos)
            )

        self.get_logger().info(
            f'Swarm TF online: robots={self._robots} map_frame={self._map_frame} '
            f'ns_tf={sorted(self._ns_tf_pubs.keys())}'
        )

    def _build_static_transforms(self) -> list[TransformStamped]:
        out: list[TransformStamped] = []

        for r in self._robots:
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp.sec = 0
            t_map_odom.header.stamp.nanosec = 0
            t_map_odom.header.frame_id = self._map_frame
            t_map_odom.child_frame_id = f'{r}/odom'
            t_map_odom.transform.rotation.w = 1.0
            out.append(t_map_odom)

            t_base = TransformStamped()
            t_base.header.stamp.sec = 0
            t_base.header.stamp.nanosec = 0
            t_base.header.frame_id = f'{r}/base_footprint'
            t_base.child_frame_id = f'{r}/base_link'
            t_base.transform.rotation.w = 1.0
            out.append(t_base)

        if self._publish_alias:
            alias = TransformStamped()
            alias.header.stamp.sec = 0
            alias.header.stamp.nanosec = 0
            alias.header.frame_id = 'robot/base_link'
            alias.child_frame_id = 'base_link'
            alias.transform.rotation.w = 1.0
            out.append(alias)

        return out

    def _publish_static(self, transforms: list[TransformStamped]) -> None:
        msg = TFMessage()
        msg.transforms = transforms
        self._tf_static_pub.publish(msg)

        for r, pub in self._ns_tf_static_pubs.items():
            filtered = _filter_transforms(transforms, prefix=r)
            if not filtered:
                continue
            out = TFMessage()
            out.transforms = filtered
            pub.publish(out)

    def _mk_odom_cb(self, robot: str):
        parent = f'{robot}/odom'
        child = f'{robot}/base_footprint'
        ns_pub = self._ns_tf_pubs.get(robot)

        def cb(msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

            t = TransformStamped()
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                t.header.stamp = self.get_clock().now().to_msg()
            else:
                t.header.stamp = msg.header.stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(p.x)
            t.transform.translation.y = float(p.y)
            t.transform.translation.z = 0.0
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            out = TFMessage()
            out.transforms = [t]
            self._tf_pub.publish(out)
            if ns_pub is not None:
                ns_pub.publish(out)

        return cb


def main() -> None:
    rclpy.init()
    node = SwarmTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['SwarmTFBroadcaster', 'main']
