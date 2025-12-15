#!/usr/bin/env python3
from __future__ import annotations
import math
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

def _qos_sensor_data(
    depth: int = 10,
    reliability: ReliabilityPolicy = ReliabilityPolicy.BEST_EFFORT,
) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )

class ScanMerger(Node):
    def __init__(self) -> None:
        super().__init__('scan_merger')
        self.declare_parameter('input_topics', [
            '/scan/front','/scan/front_right','/scan/right','/scan/back_right',
            '/scan/back','/scan/back_left','/scan/left','/scan/front_left',
        ])
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('output_frame', 'robot/base_footprint')
        self.declare_parameter('apply_yaw_offsets', True)
        self.declare_parameter(
            'yaw_offsets_deg',
            [],
        )
        self.declare_parameter('stamp_policy', 'min')      # min|max|front
        self.declare_parameter('ring_min', -math.pi)
        self.declare_parameter('ring_max',  math.pi)
        self.declare_parameter('ring_increment', 0.0)      # 0.0 => auto
        self.declare_parameter('publish_hz', 15.0)

        topics = list(self.get_parameter('input_topics').get_parameter_value().string_array_value)
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.apply_yaw_offsets = bool(self.get_parameter('apply_yaw_offsets').value)
        self.stamp_policy = self.get_parameter('stamp_policy').get_parameter_value().string_value
        self.ring_min = float(self.get_parameter('ring_min').get_parameter_value().double_value)
        self.ring_max = float(self.get_parameter('ring_max').get_parameter_value().double_value)
        self.cfg_ring_inc = float(self.get_parameter('ring_increment').get_parameter_value().double_value)
        publish_hz = float(self.get_parameter('publish_hz').get_parameter_value().double_value)

        self._topics = topics
        self._latest: Dict[str, LaserScan] = {}
        self._seen: Dict[str, bool] = {t: False for t in topics}
        self._yaw_offsets_rad = self._build_yaw_offsets(topics)

        qos = _qos_sensor_data(depth=10)
        for t in topics:
            self.create_subscription(LaserScan, t, self._mk_cb(t), qos)

        self.pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            _qos_sensor_data(depth=5),
        )
        self.timer = self.create_timer(1.0 / max(1e-3, publish_hz), self._on_timer)

        self.get_logger().info(
            f'Merging {len(topics)} scans -> {self.output_topic} '
            f'(frame="{self.output_frame}", ring=[{self.ring_min:.3f},{self.ring_max:.3f}))'
        )

    def _build_yaw_offsets(self, topics: List[str]) -> Dict[str, float]:
        if not self.apply_yaw_offsets:
            return {t: 0.0 for t in topics}

        default_deg = {
            'front': 0.0,
            'front_right': 45.0,
            'right': 90.0,
            'back_right': 135.0,
            'back': 180.0,
            'back_left': -135.0,
            'left': -90.0,
            'front_left': -45.0,
        }
        overrides_deg = list(
            self.get_parameter('yaw_offsets_deg').get_parameter_value().string_array_value
        )
        overrides_rad: Dict[str, float] = {}
        for raw in overrides_deg:
            item = str(raw).strip()
            if not item:
                continue
            if ':' in item:
                key, value = item.split(':', 1)
            elif '=' in item:
                key, value = item.split('=', 1)
            else:
                self.get_logger().warning(
                    f'Ignoring malformed yaw offset "{item}" (use topic:deg or name:deg).'
                )
                continue
            key = key.strip()
            try:
                deg = float(value.strip())
            except ValueError:
                self.get_logger().warning(
                    f'Ignoring yaw offset "{item}" (could not parse degrees).'
                )
                continue
            overrides_rad[key] = math.radians(deg)

        out: Dict[str, float] = {}
        for topic in topics:
            suffix = topic.rsplit('/', 1)[-1]
            if topic in overrides_rad:
                out[topic] = overrides_rad[topic]
            elif suffix in overrides_rad:
                out[topic] = overrides_rad[suffix]
            else:
                out[topic] = math.radians(default_deg.get(suffix, 0.0))
        return out

    def _mk_cb(self, topic: str):
        def _cb(msg: LaserScan):
            self._latest[topic] = msg
            self._seen[topic] = True
        return _cb

    @staticmethod
    def _time_key(stamp) -> tuple[int, int]:
        return (stamp.sec, stamp.nanosec)

    def _choose_stamp(self, scans: List[LaserScan]) -> Time:
        if self.stamp_policy == 'max':
            stamp = max(scans, key=lambda s: self._time_key(s.header.stamp)).header.stamp
        elif self.stamp_policy == 'front':
            stamp = scans[0].header.stamp
        else:
            stamp = min(scans, key=lambda s: self._time_key(s.header.stamp)).header.stamp
        return Time(seconds=stamp.sec + stamp.nanosec * 1e-9)

    def _derive_ring_inc(self, scans: List[LaserScan]) -> float:
        if self.cfg_ring_inc > 0.0:
            return self.cfg_ring_inc
        return min(max(s.angle_increment, 1e-6) for s in scans)

    def _on_timer(self) -> None:
        if not all(self._seen.values()):
            return
        scans: List[tuple[str, LaserScan]] = []
        for t in self._topics:
            s = self._latest.get(t)
            if s is None or (s.header.stamp.sec == 0 and s.header.stamp.nanosec == 0):
                return
            scans.append((t, s))
        merged = self._merge_by_binning(scans)
        if merged is not None:
            self.pub.publish(merged)

    def _merge_by_binning(self, scans: List[tuple[str, LaserScan]]) -> Optional[LaserScan]:
        scan_msgs = [s for _, s in scans]
        ring_inc = self._derive_ring_inc(scan_msgs)
        num_bins = int(math.ceil((self.ring_max - self.ring_min) / ring_inc))
        ranges = [math.inf] * num_bins
        intensities = [0.0] * num_bins
        out_range_min = math.inf
        out_range_max = 0.0

        for topic, s in scans:
            yaw_offset = self._yaw_offsets_rad.get(topic, 0.0)
            a = s.angle_min
            inc = max(s.angle_increment, 1e-6)
            out_range_min = min(out_range_min, s.range_min)
            out_range_max = max(out_range_max, s.range_max)
            for i, r in enumerate(s.ranges):
                if r is None or math.isnan(r):
                    continue
                ang = a + i * inc + yaw_offset
                span = self.ring_max - self.ring_min
                while ang < self.ring_min: ang += span
                while ang >= self.ring_max: ang -= span
                bin_idx = int((ang - self.ring_min) / ring_inc)
                if 0 <= bin_idx < num_bins and r < ranges[bin_idx]:
                    ranges[bin_idx] = r
                    if s.intensities and i < len(s.intensities):
                        intensities[bin_idx] = s.intensities[i]

        stamp = self._choose_stamp(scan_msgs)
        out = LaserScan()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.output_frame
        out.angle_min = self.ring_min
        out.angle_max = self.ring_min + ring_inc * (num_bins - 1)
        out.angle_increment = ring_inc

        ref = min(scan_msgs, key=lambda s: s.angle_increment)
        out.time_increment = ref.time_increment
        out.scan_time = ref.scan_time
        out.range_min = 0.0 if not math.isfinite(out_range_min) else max(0.0, out_range_min)
        out.range_max = 10.0 if out_range_max <= 0.0 else out_range_max
        out.ranges = ranges
        out.intensities = intensities
        return out

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
