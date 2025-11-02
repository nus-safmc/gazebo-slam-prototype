#!/usr/bin/env python3
# Merges multiple LaserScan topics into one 360° scan using sim-time stamps.

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def _qos_sensor_data(depth: int = 10) -> QoSProfile:
    # sensor_data profile: BestEffort + small queue (good for LaserScan)
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class ScanMerger(Node):
    """
    Merge N LaserScan topics into a single scan.

    Guarantees:
      • Outgoing header.frame_id == output_frame (defaults to 'robot/base_footprint').
      • Outgoing header.stamp comes from the inputs (sim time), policy=min/max/front.
      • Publishes on /scan_merged (underscore) to match your RViz config.
    """

    def __init__(self) -> None:
        super().__init__('scan_merger')

        # Parameters
        self.declare_parameter('input_topics', [
            '/scan/front',
            '/scan/front_right',
            '/scan/right',
            '/scan/back_right',
            '/scan/back',
            '/scan/back_left',
            '/scan/left',
            '/scan/front_left',
        ])
        # NOTE: default changed to underscore to match your RViz (stops you from
        # accidentally visualizing a different topic that still uses base_link).
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('output_frame', 'robot/base_footprint')
        self.declare_parameter('stamp_policy', 'min')        # 'min'|'max'|'front'
        self.declare_parameter('angle_increment_tolerance', 1e-6)
        self.declare_parameter('merge_rate_hz', 15.0)

        topics_param = self.get_parameter('input_topics').get_parameter_value().string_array_value
        if not topics_param:
            raise ValueError('scan_merger requires at least one input topic')
        topics: List[str] = list(topics_param)
        self.output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_frame: str = self.get_parameter('output_frame').get_parameter_value().string_value
        self.stamp_policy: str = self.get_parameter('stamp_policy').get_parameter_value().string_value
        self.inc_tol: float = self.get_parameter('angle_increment_tolerance').get_parameter_value().double_value
        merge_rate = self.get_parameter('merge_rate_hz').get_parameter_value().double_value

        # Latest scan per topic
        self._latest: Dict[str, LaserScan] = {}
        self._seen: Dict[str, bool] = {t: False for t in topics}
        self._topics: List[str] = topics

        qos = _qos_sensor_data(depth=10)
        for t in topics:
            self.create_subscription(LaserScan, t, self._mk_cb(t), qos)

        self.pub = self.create_publisher(LaserScan, self.output_topic, _qos_sensor_data(depth=5))
        self.timer = self.create_timer(1.0 / max(1e-3, merge_rate), self._on_timer)

        self._pub_count = 0
        self.get_logger().info(
            f'Merging {len(topics)} scans -> {self.output_topic} '
            f'(frame="{self.output_frame}", policy={self.stamp_policy})'
        )

    def _mk_cb(self, topic: str):
        def _cb(msg: LaserScan):
            self._latest[topic] = msg
            self._seen[topic] = True
        return _cb

    def _choose_stamp(self, scans: List[LaserScan]) -> Time:
        # Choose a realistic stamp from inputs (sim-time from /clock).
        if self.stamp_policy == 'max':
            base = max(scans, key=lambda s: (s.header.stamp.sec, s.header.stamp.nanosec)).header.stamp
        elif self.stamp_policy == 'front':
            base = scans[0].header.stamp
        else:
            base = min(scans, key=lambda s: (s.header.stamp.sec, s.header.stamp.nanosec)).header.stamp
        return Time(seconds=base.sec + base.nanosec * 1e-9)

    @staticmethod
    def _equal(a: float, b: float, tol: float) -> bool:
        return abs(a - b) <= tol

    def _validate_geometry(self, scans: List[LaserScan]) -> Tuple[float, int]:
        first = scans[0]
        inc = first.angle_increment
        total_n = 0
        for s in scans:
            if not self._equal(s.angle_increment, inc, self.inc_tol):
                self.get_logger().warn(
                    f'angle_increment mismatch: {s.angle_increment} vs {inc}. '
                    f'Resampling not implemented; using first increment.'
                )
            total_n += len(s.ranges)
        return inc, total_n

    def _merge(self, scans: List[LaserScan]) -> Optional[LaserScan]:
        if not scans:
            return None

        inc, _ = self._validate_geometry(scans)
        merged_stamp = self._choose_stamp(scans)

        # Build contiguous angles by chaining segments in self._topics order
        angle_min = scans[0].angle_min
        angle_max = angle_min
        for s in scans:
            seg_span = s.angle_increment * (len(s.ranges) - 1)
            angle_max = angle_max + seg_span + inc  # continue from next step

        ranges: List[float] = []
        intensities: List[float] = []
        for s in scans:
            ranges.extend(s.ranges)
            intensities.extend(s.intensities if s.intensities else [0.0] * len(s.ranges))

        first = scans[0]
        out = LaserScan()
        out.header.stamp = merged_stamp.to_msg()            # sim time from inputs
        out.header.frame_id = self.output_frame             # MUST exist in TF

        out.angle_min = angle_min
        out.angle_max = angle_min + inc * (len(ranges) - 1)
        out.angle_increment = inc

        out.time_increment = first.time_increment
        out.scan_time = first.scan_time
        out.range_min = min(s.range_min for s in scans)
        out.range_max = max(s.range_max for s in scans)
        out.ranges = ranges
        out.intensities = intensities
        return out

    def _on_timer(self) -> None:
        if not all(self._seen.values()):
            return

        scans: List[LaserScan] = []
        for t in self._topics:
            s = self._latest.get(t)
            if s is None:
                return
            if s.header.stamp.sec == 0 and s.header.stamp.nanosec == 0:
                return
            scans.append(s)

        merged = self._merge(scans)
        if merged is not None:
            # Belt & suspenders: enforce frame & stamp again before publish.
            merged.header.frame_id = self.output_frame
            if merged.header.stamp.sec == 0 and merged.header.stamp.nanosec == 0:
                merged.header.stamp = self.get_clock().now().to_msg()  # fallback
            self.pub.publish(merged)

            # Light debug every ~2s
            self._pub_count += 1
            if self._pub_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.output_topic}: frame={merged.header.frame_id} '
                    f'stamp={merged.header.stamp.sec + merged.header.stamp.nanosec*1e-9:.3f}'
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMerger()
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
