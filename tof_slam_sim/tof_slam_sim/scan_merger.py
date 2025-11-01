#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from typing import Dict, Optional

class ScanMerger(Node):
    """Merges 8 individual LaserScans into a single 360° scan."""
    
    def __init__(self):
        super().__init__('scan_merger')
        
        self.base_frame = self.declare_parameter(
            'base_frame',
            'robot/base_footprint'
        ).value

        # Sensor parameters
        self.sensor_names = ['front', 'front_right', 'right', 'back_right',
                           'back', 'back_left', 'left', 'front_left']
        self.sensor_angles = [0.0, math.pi/4, math.pi/2, 3*math.pi/4,
                            math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
        
        # Track latest scan per sensor; publish even if one sensor skips a frame.
        self.latest_scans: Dict[str, Optional[LaserScan]] = {
            name: None for name in self.sensor_names
        }
        self.received_once = set()

        for name in self.sensor_names:
            self.create_subscription(
                LaserScan,
                f'/scan/{name}',
                lambda msg, name=name: self.scan_callback(name, msg),
                10,
            )

        # Publish at ~10 Hz using the freshest data available.
        self.publish_timer = self.create_timer(0.1, self.publish_merged)
        
        # Create merged scan publisher
        self.merged_pub = self.create_publisher(
            LaserScan,
            '/scan_merged',
            10
        )
        
        self.get_logger().info('Scan merger initialized')
    
    def scan_callback(self, name: str, msg: LaserScan) -> None:
        self.latest_scans[name] = msg
        self.received_once.add(name)

    def publish_merged(self) -> None:
        if not self.received_once:
            return

        try:
            template_scan: Optional[LaserScan] = None
            latest_time = None
            for scan in self.latest_scans.values():
                if scan is None:
                    continue
                if latest_time is None or scan.header.stamp > latest_time:
                    latest_time = scan.header.stamp
                    template_scan = scan

            if template_scan is None:
                return

            merged_scan = LaserScan()
            merged_scan.header = template_scan.header
            merged_scan.header.frame_id = self.base_frame
            merged_scan.angle_min = -math.pi
            merged_scan.angle_max = math.pi
            merged_scan.angle_increment = math.pi / 32.0  # 64 points total
            merged_scan.time_increment = template_scan.time_increment
            merged_scan.scan_time = template_scan.scan_time
            merged_scan.range_min = template_scan.range_min
            merged_scan.range_max = template_scan.range_max

            num_points = int(2 * math.pi / merged_scan.angle_increment)
            ranges = [float('inf')] * num_points

            for scan_msg, sensor_angle in zip(self.latest_scans.values(), self.sensor_angles):
                if scan_msg is None:
                    continue

                scan_angle_min = scan_msg.angle_min + sensor_angle
                scan_increment = scan_msg.angle_increment

                for i, r in enumerate(scan_msg.ranges):
                    if not math.isfinite(r):
                        continue

                    angle = scan_angle_min + i * scan_increment

                    while angle > math.pi:
                        angle -= 2 * math.pi
                    while angle < -math.pi:
                        angle += 2 * math.pi

                    idx = int((angle - merged_scan.angle_min) /
                              merged_scan.angle_increment)
                    if 0 <= idx < num_points:
                        ranges[idx] = min(ranges[idx], r)

            merged_scan.ranges = ranges
            self.merged_pub.publish(merged_scan)

        except Exception as exc:
            self.get_logger().error(f'Error merging scans: {exc}')

def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
