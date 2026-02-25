#!/usr/bin/env python3
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class _LatestPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class GzPoseInfoToPoseStamped(Node):
    """Publish a Gazebo model pose (from /world/<world>/dynamic_pose/info) as ROS PoseStamped."""

    def __init__(self) -> None:
        super().__init__('gz_pose_info_to_pose_stamped')

        self.declare_parameter('gz_world_name', 'playfield')
        self.declare_parameter('gz_topic', '')
        self.declare_parameter('entity_name', 'x500_small_tof_0')
        self.declare_parameter('pose_topic', '/model/x500_small_tof_0/pose')
        self.declare_parameter('pose_frame_id', 'world')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('warn_period_sec', 5.0)

        world = str(self.get_parameter('gz_world_name').value).strip() or 'playfield'
        gz_topic = str(self.get_parameter('gz_topic').value).strip()
        if not gz_topic:
            gz_topic = f'/world/{world}/dynamic_pose/info'

        self._entity = str(self.get_parameter('entity_name').value).strip() or 'x500_small_tof_0'
        self._pose_topic = str(self.get_parameter('pose_topic').value).strip() or '/model/x500_small_tof_0/pose'
        self._frame_id = str(self.get_parameter('pose_frame_id').value).strip() or 'world'

        publish_hz = float(self.get_parameter('publish_hz').value)
        publish_hz = max(1.0, publish_hz)
        self._publish_period_sec = 1.0 / publish_hz

        warn_period_sec = float(self.get_parameter('warn_period_sec').value)
        self._warn_period_ns = int(max(0.0, warn_period_sec) * 1e9)
        self._last_warn_ns: Optional[int] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PoseStamped, self._pose_topic, qos)

        self._lock = threading.Lock()
        self._latest: Optional[_LatestPose] = None

        self._gz = GzNode()
        ok = self._gz.subscribe(Pose_V, gz_topic, self._on_pose_v)
        if not ok:
            raise RuntimeError(f'Failed to subscribe to Gazebo topic: {gz_topic}')

        self.create_timer(self._publish_period_sec, self._tick)

        self.get_logger().info(
            f'Gazebo pose bridge online: {gz_topic} -> {self._pose_topic} '
            f'(entity=\"{self._entity}\", frame_id=\"{self._frame_id}\", {publish_hz:.1f} Hz)'
        )

    def _on_pose_v(self, msg: Pose_V) -> None:
        for pose in msg.pose:
            if pose.name != self._entity:
                continue
            p = pose.position
            q = pose.orientation
            latest = _LatestPose(
                x=float(p.x),
                y=float(p.y),
                z=float(p.z),
                qx=float(q.x),
                qy=float(q.y),
                qz=float(q.z),
                qw=float(q.w),
            )
            with self._lock:
                self._latest = latest
            return

    def _tick(self) -> None:
        with self._lock:
            latest = self._latest

        if latest is None:
            if self._warn_period_ns <= 0:
                return
            now_ns = int(self.get_clock().now().nanoseconds)
            should_warn = self._last_warn_ns is None or (now_ns - self._last_warn_ns) >= self._warn_period_ns
            if should_warn:
                self._last_warn_ns = now_ns
                self.get_logger().warn(f'No Gazebo pose received yet for entity \"{self._entity}\".')
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = latest.x
        msg.pose.position.y = latest.y
        msg.pose.position.z = latest.z
        msg.pose.orientation.x = latest.qx
        msg.pose.orientation.y = latest.qy
        msg.pose.orientation.z = latest.qz
        msg.pose.orientation.w = latest.qw
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GzPoseInfoToPoseStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['GzPoseInfoToPoseStamped', 'main']

