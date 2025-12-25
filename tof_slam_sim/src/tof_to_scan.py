#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import math
from typing import Optional


class TofToScan(Node):
    #Concatenates depth images from all ToF sensors, publishes as single LaserScan message

    def __init__(self):
        super().__init__('TofToScan')

        self.output_frame = (
            self.declare_parameter('output_frame', 'robot/base_link')
            .get_parameter_value()
            .string_value
        )
        self.output_topic = (
            self.declare_parameter('output_topic', '/scan_merged')
            .get_parameter_value()
            .string_value
        )
        self.viz_output_topic = (
            self.declare_parameter('viz_output_topic', '')
            .get_parameter_value()
            .string_value
            .strip()
        )
        publish_hz = (
            self.declare_parameter('publish_hz', 10.0)
            .get_parameter_value()
            .double_value
        )
        self.publish_hz = float(publish_hz)
        if self.publish_hz <= 0.0:
            self.publish_hz = 10.0

        # QoS profile for depth data
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.h_fov = math.pi/4.0
        self.v_fov = math.pi/4.0
        self.ring_radius_m = float(
            self.declare_parameter('ring_radius_m', 0.05).get_parameter_value().double_value
        )
        self.minrange = float(
            self.declare_parameter('min_range_m', 0.25).get_parameter_value().double_value
        )
        self.maxrange = float(
            self.declare_parameter('max_range_m', 6.0).get_parameter_value().double_value
        )
        self.minrange = max(0.0, self.minrange)
        self.maxrange = max(self.minrange + 0.01, self.maxrange)
        self.roi_row_start = int(self.declare_parameter('roi_row_start', 2).value)
        self.roi_row_end = int(self.declare_parameter('roi_row_end', 6).value)
        self.column_reduce = (
            self.declare_parameter('column_reduce', 'min')
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )

        self.cv_bridge = CvBridge()
        self._scaled_mm_sensors: set[int] = set()

        self._latest: list[Optional[Image]] = [None] * 8

        for i in range(8):
            topic = f'/depth/tof_{i+1}'

            def _make_cb(index: int):
                def _cb(msg: Image) -> None:
                    self._latest[index] = msg

                return _cb

            self.create_subscription(Image, topic, _make_cb(i), depth_qos)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, scan_qos)
        self.viz_pub = None
        if self.viz_output_topic:
            self.viz_pub = self.create_publisher(LaserScan, self.viz_output_topic, scan_qos)

        period_sec = 1.0 / max(0.1, self.publish_hz)
        self.create_timer(period_sec, self._publish_scan)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _publish_scan(self) -> None:
        images = self._latest

        try:
            merged_scan = LaserScan()
            # Stamp with the oldest contributing sensor image stamp to reduce TF extrapolation
            # into the future (a common cause of message_filter queue overflows).
            oldest_ns: Optional[int] = None
            oldest_stamp = None
            for img in images:
                if img is None:
                    continue
                stamp = img.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    continue
                stamp_ns = self._stamp_ns(stamp)
                if oldest_ns is None or stamp_ns < oldest_ns:
                    oldest_ns = stamp_ns
                    oldest_stamp = stamp

            merged_scan.header.stamp = (
                oldest_stamp if oldest_stamp is not None else self.get_clock().now().to_msg()
            )
            merged_scan.header.frame_id = self.output_frame
            merged_scan.angle_min = 0.0

            num_points = 64
            merged_scan.angle_increment = (2.0 * math.pi) / float(num_points)
            merged_scan.angle_max = merged_scan.angle_min + merged_scan.angle_increment * (num_points - 1)
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = 1.0 / max(0.1, self.publish_hz) if self.publish_hz > 0.0 else 1.0 / 30.0
            merged_scan.range_min = self.minrange
            # Ranges are expressed from the base_link origin, but individual sensors are offset
            # from the origin by ring_radius_m. Account for that so valid points near the
            # sensor far-clip remain within [range_min, range_max].
            merged_scan.range_max = self.maxrange + self.ring_radius_m

            ranges = [float('inf')] * num_points

            for sensor_index in range(8):
                depth_image = images[sensor_index]
                if depth_image is None:
                    continue
                sensor_angle = sensor_index * (math.pi / 4.0)
                sx = self.ring_radius_m * math.cos(sensor_angle)
                sy = self.ring_radius_m * math.sin(sensor_angle)
                image = self.cv_bridge.imgmsg_to_cv2(depth_image)
                rows = int(image.shape[0]) if hasattr(image, 'shape') else 0
                row_start = max(0, min(rows, int(self.roi_row_start)))
                row_end = max(row_start + 1, min(rows, int(self.roi_row_end)))

                # Some Gazebo depth images come through as millimeters (e.g. uint16),
                # others are already meters (float32). Auto-detect by magnitude.
                roi = image[row_start:row_end, :8] if rows > 0 else image
                try:
                    finite_roi = roi[np.isfinite(roi)]
                    if finite_roi.size > 0:
                        med = float(np.median(finite_roi))
                        if med > (self.maxrange * 10.0):
                            image = image.astype(np.float32) * 0.001
                            if sensor_index not in self._scaled_mm_sensors:
                                self._scaled_mm_sensors.add(sensor_index)
                                self.get_logger().info(
                                    f'/depth/tof_{sensor_index+1} appears to be in millimeters; scaling to meters.'
                                )
                except Exception:
                    pass

                img_ranges = []
                for col in range(8):
                    col_depths = image[row_start:row_end, col]
                    valid_depths = col_depths[
                        (col_depths >= self.minrange) & 
                        (col_depths <= self.maxrange)
                    ]
                    if len(valid_depths) > 0:
                        reduce = self.column_reduce
                        if reduce == 'max':
                            img_ranges.append(float(np.max(valid_depths)))
                        elif reduce == 'median':
                            img_ranges.append(float(np.median(valid_depths)))
                        elif reduce.startswith('p') and reduce[1:].isdigit():
                            p = max(0, min(100, int(reduce[1:])))
                            img_ranges.append(float(np.percentile(valid_depths, p)))
                        else:
                            img_ranges.append(float(np.min(valid_depths)))
                    else:
                        img_ranges.append(float('inf'))
                
                # Map each range to the merged scan
                for col_index, r in enumerate(img_ranges):
                    if not math.isfinite(r):
                        continue

                    # Gazebo's depth camera reports values at the far clip for "no return".
                    # Treat that as infinity so Nav2 clears space without marking a fake wall.
                    if r >= (self.maxrange - 1e-3):
                        continue

                    # Horizontal beam angle relative to this sensor (centered bins).
                    theta_rel = (self.h_fov * 0.5) - (float(col_index) + 0.5) * (self.h_fov / 8.0)
                    theta = sensor_angle + theta_rel

                    # Normalize to [0, 2*pi)
                    while theta < 0.0:
                        theta += 2.0 * math.pi
                    while theta >= 2.0 * math.pi:
                        theta -= 2.0 * math.pi

                    # Project the hit point into base_link frame and compute range from base origin.
                    px = sx + float(r) * math.cos(theta)
                    py = sy + float(r) * math.sin(theta)
                    r_center = math.hypot(px, py)

                    idx = int((theta - merged_scan.angle_min) / merged_scan.angle_increment)
                    if 0 <= idx < num_points:
                        ranges[idx] = min(ranges[idx], r_center)

            merged_scan.ranges = ranges
            self.pub.publish(merged_scan)

            if self.viz_pub is not None:
                viz = LaserScan()
                viz.header = merged_scan.header
                viz.header.frame_id = merged_scan.header.frame_id
                viz.angle_min = merged_scan.angle_min
                viz.angle_max = merged_scan.angle_max
                viz.angle_increment = merged_scan.angle_increment
                viz.time_increment = merged_scan.time_increment
                viz.scan_time = merged_scan.scan_time
                viz.range_min = merged_scan.range_min
                viz.range_max = merged_scan.range_max
                # RViz only draws finite ranges; substitute "no return" with range_max for display.
                viz.ranges = [
                    (viz.range_max if (not math.isfinite(val)) else float(val)) for val in merged_scan.ranges
                ]
                viz.intensities = list(merged_scan.intensities)
                self.viz_pub.publish(viz)
        
        except Exception as e:
            self.get_logger().error(f'Error merging scans: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TofToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
