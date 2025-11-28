#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import numpy as np
import math
from typing import List, Tuple


class TofToScan(Node):
    #Concatenates depth images from all ToF sensors, publishes as single LaserScan message

    def __init__(self):
        super().__init__('TofToScan')

        # QoS profile for depth data
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.h_fov = math.pi/4.0
        self.v_fov = math.pi/4.0
        self.minrange = 0.05
        self.maxrange = 4.0

        self.cv_bridge = CvBridge()
        
        self.subs = []
        for i in range (8):
            sub = Subscriber(self, Image, f'/depth/tof_{i+1}')
            self.subs.append(sub)

        self.ts = ApproximateTimeSynchronizer(
            self.subs, queue_size=8, slop=0.1
        )
        self.ts.registerCallback(self.depth_callback)

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_merged',
            10
        )        
        
    def depth_callback(self, *images: List[Image]):
        try:
            merged_scan = LaserScan()
            merged_scan.header = images[0].header
            merged_scan.angle_min = 0
            merged_scan.angle_max = 2 * math.pi
            merged_scan.angle_increment = math.pi / 32.0  # pi/4 /8
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = 1.0/30.0
            merged_scan.range_min = self.minrange
            merged_scan.range_max = self.maxrange

            num_points = 64
            ranges = [float('inf')] * num_points

            for i in range(8):
                depth_image = images[i]
                sensor_angle = i*(math.pi/4)
                scan_angle_min = sensor_angle - math.pi/8
                scan_angle_max = sensor_angle + math.pi/8
                image = self.cv_bridge.imgmsg_to_cv2(depth_image)
                img_ranges = []
                for col in range(8):
                    col_depths = image[:, col]
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
                for i, r in enumerate(img_ranges):
                    if not math.isinf(r):
                        # Calculate angle in merged scan frame
                        angle = scan_angle_max - i * scan_increment
                        #Calculate position offset from origin
                        r_angle = math.pi/8 - i * math.pi/32 - math.pi/64

                        #Sensor is offset from centre of drone by 0.05m (radius of ToF-Ring)
                        #Depth camera returns direct projected distance onto its image plane 
                        offset_r = math.sqrt((0.05 + r)**2 + (r*math.tan(r_angle))**2) # r' = sqrt((x + r)^2 + (rtana)^2)
                        
                        # Normalize angle to [0, 2*pi]
                        while angle < 0:
                            angle += 2 * math.pi
                        while angle > (2 * math.pi):
                            angle -= 2 * math.pi 
                        
                        # Find corresponding index in merged scan
                        idx = int(round((angle - merged_scan.angle_min) / 
                                merged_scan.angle_increment))
                        
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
