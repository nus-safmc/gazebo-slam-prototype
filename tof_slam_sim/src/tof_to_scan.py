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

        period_sec = 1.0 / max(0.1, self.publish_hz)
        self.create_timer(period_sec, self._publish_scan)

    def _publish_scan(self) -> None:
        images = self._latest

        try:
            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = self.output_frame
            merged_scan.angle_min = 0.0

            num_points = 64
            merged_scan.angle_increment = (2.0 * math.pi) / float(num_points)
            merged_scan.angle_max = merged_scan.angle_min + merged_scan.angle_increment * (num_points - 1)
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = 1.0 / max(0.1, self.publish_hz) if self.publish_hz > 0.0 else 1.0 / 30.0
            merged_scan.range_min = self.minrange
            merged_scan.range_max = self.maxrange

            ranges = [float('inf')] * num_points

            for sensor_index in range(8):
                depth_image = images[sensor_index]
                if depth_image is None:
                    continue
                sensor_angle = sensor_index * (math.pi / 4)
                scan_angle_min = sensor_angle - math.pi/8
                scan_angle_max = sensor_angle + math.pi/8
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
                        img_ranges.append(float(np.min(valid_depths)))
                    else:
                        img_ranges.append(float('inf'))
                
                scan_increment = merged_scan.angle_increment
                
                # Map each range to the merged scan
                for col_index, r in enumerate(img_ranges):
                    if not math.isinf(r):
                        # Calculate angle in merged scan frame
                        angle = scan_angle_max - col_index * scan_increment
                        #Calculate position offset from origin
                        r_angle = math.pi/8 - col_index * math.pi/32 - math.pi/64

                        # Sensor is offset from centre of drone by ring radius.
                        #Depth camera returns direct projected distance onto its image plane 
                        offset_r = math.sqrt((self.ring_radius_m + r)**2 + (r*math.tan(r_angle))**2) # r' = sqrt((x + r)^2 + (rtana)^2)
                        
                        # Normalize angle to [0, 2*pi]
                        while angle < 0:
                            angle += 2 * math.pi
                        while angle > (2 * math.pi):
                            angle -= 2 * math.pi 
                        
                        # Find corresponding index in merged scan
                        idx = int((angle - merged_scan.angle_min) / merged_scan.angle_increment)
                        
                        if 0 <= idx < num_points:
                            # Take minimum valid range
                            ranges[idx] = min(ranges[idx], offset_r)

            merged_scan.ranges = ranges
            self.pub.publish(merged_scan)
        
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
