#!/usr/bin/env python3
"""Publish a fixed-size /map covering a known arena region.

Nav2 costmaps and planners can fail if the SLAM /map starts as 0×0 or if the
dynamic SLAM map window doesn't cover the full playfield. This node publishes a
stable OccupancyGrid for /map (unknown everywhere by default) and embeds an
incoming SLAM map into it.
"""

from __future__ import annotations

from array import array
import math

import rclpy
from geometry_msgs.msg import Pose
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _identity_pose(x: float, y: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    return pose


class ArenaMapPadder(Node):
    def __init__(self) -> None:
        super().__init__('arena_map_padder')

        self.declare_parameter('arena_min_x', -20.0)
        self.declare_parameter('arena_max_x', 20.0)
        self.declare_parameter('arena_min_y', -20.0)
        self.declare_parameter('arena_max_y', 20.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('input_map_topic', '/slam_map')
        self.declare_parameter('input_update_topic', '/slam_map_updates')
        self.declare_parameter('output_map_topic', '/map')
        self.declare_parameter('output_update_topic', '/map_updates')
        self.declare_parameter('frame_id', 'robot/map')
        self.declare_parameter('publish_period_sec', 1.0)

        arena_min_x = float(self.get_parameter('arena_min_x').value)
        arena_max_x = float(self.get_parameter('arena_max_x').value)
        arena_min_y = float(self.get_parameter('arena_min_y').value)
        arena_max_y = float(self.get_parameter('arena_max_y').value)
        self._default_resolution = float(self.get_parameter('resolution').value)
        self._input_map_topic = str(self.get_parameter('input_map_topic').value)
        self._input_update_topic = str(self.get_parameter('input_update_topic').value).strip()
        self._output_map_topic = str(self.get_parameter('output_map_topic').value)
        self._output_update_topic = str(self.get_parameter('output_update_topic').value).strip()
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self._arena_min_x = min(arena_min_x, arena_max_x)
        self._arena_max_x = max(arena_min_x, arena_max_x)
        self._arena_min_y = min(arena_min_y, arena_max_y)
        self._arena_max_y = max(arena_min_y, arena_max_y)

        self._resolution = self._default_resolution
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._width = 0
        self._height = 0
        self._blank_data: array | None = None
        self._latest_map: OccupancyGrid | None = None
        self._last_input_offset_x = 0
        self._last_input_offset_y = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(OccupancyGrid, self._output_map_topic, qos)
        self._sub = self.create_subscription(OccupancyGrid, self._input_map_topic, self._on_input_map, qos)

        update_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._update_pub = None
        self._update_sub = None
        if self._input_update_topic and self._output_update_topic:
            self._update_pub = self.create_publisher(OccupancyGridUpdate, self._output_update_topic, update_qos)
            self._update_sub = self.create_subscription(
                OccupancyGridUpdate,
                self._input_update_topic,
                self._on_input_update,
                update_qos,
            )

        if self._publish_period_sec <= 0.0:
            self._publish_period_sec = 1.0
        self._timer = self.create_timer(self._publish_period_sec, self._on_timer)

        self._build_grid(self._default_resolution)
        self._latest_map = self._make_blank_map(self._frame_id)
        self._pub.publish(self._latest_map)
        update_in = self._input_update_topic or '(disabled)'
        update_out = self._output_update_topic or '(disabled)'
        self.get_logger().info(
            f'Publishing padded map on {self._output_map_topic} '
            f'from {self._input_map_topic} (updates {update_in} -> {update_out}, '
            f'arena x=[{self._arena_min_x:.2f},{self._arena_max_x:.2f}] '
            f'y=[{self._arena_min_y:.2f},{self._arena_max_y:.2f}] res={self._resolution:.3f}).'
        )

    def _build_grid(self, resolution: float) -> None:
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError(f'Invalid resolution: {resolution}')

        origin_x = math.floor(self._arena_min_x / resolution) * resolution
        origin_y = math.floor(self._arena_min_y / resolution) * resolution
        width = int(math.ceil((self._arena_max_x - origin_x) / resolution))
        height = int(math.ceil((self._arena_max_y - origin_y) / resolution))

        if width <= 0 or height <= 0:
            raise ValueError(f'Invalid arena bounds for resolution {resolution}: {width}x{height}')

        self._resolution = float(resolution)
        self._origin_x = float(origin_x)
        self._origin_y = float(origin_y)
        self._width = int(width)
        self._height = int(height)
        self._blank_data = array('b', [-1]) * (self._width * self._height)

        self.get_logger().info(
            f'Output /map grid: {self._width}x{self._height} @ {self._resolution:.3f}m '
            f'origin=({self._origin_x:.2f},{self._origin_y:.2f})'
        )

    def _make_blank_map(self, frame_id: str) -> OccupancyGrid:
        if self._blank_data is None:
            raise RuntimeError('Grid not initialized')

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin = _identity_pose(self._origin_x, self._origin_y)
        msg.data = array('b', self._blank_data)
        return msg

    def _on_input_map(self, msg: OccupancyGrid) -> None:
        in_w = int(msg.info.width)
        in_h = int(msg.info.height)
        expected_len = in_w * in_h
        if in_w <= 0 or in_h <= 0 or len(msg.data) != expected_len:
            self.get_logger().warn(
                f'Ignoring malformed input map on {self._input_map_topic}: '
                f'w={in_w} h={in_h} len={len(msg.data)} expected={expected_len}'
            )
            return

        in_res = float(msg.info.resolution)
        if math.isfinite(in_res) and in_res > 0.0 and abs(in_res - self._resolution) > 1e-6:
            self.get_logger().warn(
                f'Input map resolution changed ({self._resolution:.6f} -> {in_res:.6f}); rebuilding output grid.'
            )
            self._build_grid(in_res)

        if self._blank_data is None:
            return

        in_origin_x = float(msg.info.origin.position.x)
        in_origin_y = float(msg.info.origin.position.y)
        dx = int(round((in_origin_x - self._origin_x) / self._resolution))
        dy = int(round((in_origin_y - self._origin_y) / self._resolution))
        self._last_input_offset_x = dx
        self._last_input_offset_y = dy

        out_w = self._width
        out_h = self._height

        src_x0 = 0
        src_y0 = 0
        dst_x0 = dx
        dst_y0 = dy
        if dst_x0 < 0:
            src_x0 = -dst_x0
            dst_x0 = 0
        if dst_y0 < 0:
            src_y0 = -dst_y0
            dst_y0 = 0

        copy_w = min(in_w - src_x0, out_w - dst_x0)
        copy_h = min(in_h - src_y0, out_h - dst_y0)
        if copy_w <= 0 or copy_h <= 0:
            self.get_logger().warn(
                f'Input map origin ({in_origin_x:.2f},{in_origin_y:.2f}) does not overlap arena map; '
                f'offset=({dx},{dy}) in={in_w}x{in_h} out={out_w}x{out_h}'
            )
            return

        padded = array('b', self._blank_data)

        for row in range(copy_h):
            src_row = (src_y0 + row) * in_w + src_x0
            dst_row = (dst_y0 + row) * out_w + dst_x0
            padded[dst_row : dst_row + copy_w] = msg.data[src_row : src_row + copy_w]

        out = OccupancyGrid()
        out.header.frame_id = msg.header.frame_id or self._frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = self._resolution
        out.info.width = out_w
        out.info.height = out_h
        out.info.origin = _identity_pose(self._origin_x, self._origin_y)
        out.data = padded

        self._latest_map = out

    def _on_input_update(self, msg: OccupancyGridUpdate) -> None:
        if self._update_pub is None:
            return
        if self._latest_map is None:
            return

        in_w = int(msg.width)
        in_h = int(msg.height)
        expected_len = in_w * in_h
        if in_w <= 0 or in_h <= 0 or len(msg.data) != expected_len:
            self.get_logger().warn(
                f'Ignoring malformed input map update on {self._input_update_topic}: '
                f'w={in_w} h={in_h} len={len(msg.data)} expected={expected_len}'
            )
            return

        out_w = self._width
        out_h = self._height

        dx = int(self._last_input_offset_x)
        dy = int(self._last_input_offset_y)

        dst_x0 = dx + int(msg.x)
        dst_y0 = dy + int(msg.y)

        src_x0 = 0
        src_y0 = 0
        if dst_x0 < 0:
            src_x0 = -dst_x0
            dst_x0 = 0
        if dst_y0 < 0:
            src_y0 = -dst_y0
            dst_y0 = 0

        copy_w = min(in_w - src_x0, out_w - dst_x0)
        copy_h = min(in_h - src_y0, out_h - dst_y0)
        if copy_w <= 0 or copy_h <= 0:
            return

        data = array('b')
        for row in range(copy_h):
            src_row = (src_y0 + row) * in_w + src_x0
            data.extend(msg.data[src_row : src_row + copy_w])

        out = OccupancyGridUpdate()
        out.header.frame_id = self._frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.x = int(dst_x0)
        out.y = int(dst_y0)
        out.width = int(copy_w)
        out.height = int(copy_h)
        out.data = data
        self._update_pub.publish(out)

        # Keep the full-map cache in sync so consumers that ignore updates still work.
        latest = self._latest_map
        if latest is None:
            return
        base = array('b', latest.data)
        for row in range(copy_h):
            dst_row = (dst_y0 + row) * out_w + dst_x0
            src_row = row * copy_w
            base[dst_row : dst_row + copy_w] = data[src_row : src_row + copy_w]
        latest.data = base

    def _on_timer(self) -> None:
        if self._latest_map is None:
            return
        self._latest_map.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._latest_map)


def main() -> None:
    rclpy.init()
    node = ArenaMapPadder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
