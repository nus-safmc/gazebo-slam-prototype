#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import math
from typing import List

class ScanMerger(Node):
    """Merges 8 individual LaserScans into a single 360° scan."""
    
    def __init__(self):
        super().__init__('scan_merger')
        
        # Sensor parameters
        self.sensor_names = ['front', 'front_right', 'right', 'back_right',
                           'back', 'back_left', 'left', 'front_left']
        self.sensor_angles = [0.0, math.pi/4, math.pi/2, 3*math.pi/4,
                            math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
        
        # Create synchronized subscribers
        self.scan_subs = []
        for name in self.sensor_names:
            sub = Subscriber(self, LaserScan, f'/scan/{name}')
            self.scan_subs.append(sub)
        
        # Synchronize messages with 0.1s tolerance
        self.ts = ApproximateTimeSynchronizer(
            self.scan_subs, queue_size=5, slop=0.1
        )
        self.ts.registerCallback(self.scans_callback)
        
        # Create merged scan publisher
        self.merged_pub = self.create_publisher(
            LaserScan,
            '/scan_merged',
            10
        )
        
        self.get_logger().info('Scan merger initialized')
    
    def scans_callback(self, *scan_msgs: List[LaserScan]) -> None:
        """Merge multiple LaserScan messages into one 360° scan."""
        try:
            # Initialize merged scan parameters
            merged_scan = LaserScan()
            merged_scan.header = scan_msgs[0].header
            merged_scan.angle_min = -math.pi
            merged_scan.angle_max = math.pi
            merged_scan.angle_increment = math.pi / 32.0  # 64 points total
            merged_scan.time_increment = scan_msgs[0].time_increment
            merged_scan.scan_time = scan_msgs[0].scan_time
            merged_scan.range_min = scan_msgs[0].range_min
            merged_scan.range_max = scan_msgs[0].range_max
            
            # Initialize ranges array
            num_points = int(2 * math.pi / merged_scan.angle_increment)
            ranges = [float('inf')] * num_points
            
            # Process each sensor's scan
            for scan_msg, sensor_angle in zip(scan_msgs, self.sensor_angles):
                # Get scan points
                scan_ranges = scan_msg.ranges
                scan_angle_min = scan_msg.angle_min + sensor_angle
                scan_angle_max = scan_msg.angle_max + sensor_angle
                scan_increment = scan_msg.angle_increment
                
                # Map each range to the merged scan
                for i, r in enumerate(scan_ranges):
                    if not math.isinf(r):
                        # Calculate angle in merged scan frame
                        angle = scan_angle_min + i * scan_increment
                        
                        # Normalize angle to [-pi, pi]
                        while angle > math.pi:
                            angle -= 2 * math.pi
                        while angle < -math.pi:
                            angle += 2 * math.pi
                        
                        # Find corresponding index in merged scan
                        idx = int((angle - merged_scan.angle_min) / 
                                merged_scan.angle_increment)
                        
                        if 0 <= idx < num_points:
                            # Take minimum valid range
                            ranges[idx] = min(ranges[idx], r)
            
            # Set merged ranges
            merged_scan.ranges = ranges
            
            # Publish merged scan
            self.merged_pub.publish(merged_scan)
            
        except Exception as e:
            self.get_logger().error(f'Error merging scans: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
