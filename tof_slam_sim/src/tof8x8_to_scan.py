#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge
import numpy as np
import math
from typing import List, Tuple

class ToF8x8ToScan(Node):
    """Converts 8×8 depth images from VL53L7CX ToF sensors to LaserScan messages."""
    
    def __init__(self):
        super().__init__('tof8x8_to_scan')
        
        # QoS profile for depth data
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Sensor parameters
        self.sensor_names = ['front', 'front_right', 'right', 'back_right',
                           'back', 'back_left', 'left', 'front_left']
        self.h_fov = 1.047  # 60 degrees in radians
        self.v_fov = 0.785  # 45 degrees in radians
        self.range_min = 0.02
        self.range_max = 3.5
        
        # OpenCV bridge
        self.cv_bridge = CvBridge()
        
        # Create subscribers and publishers for each sensor
        self.depth_subs = []
        self.scan_pubs = []
        
        for name in self.sensor_names:
            # Subscribe to depth image
            sub = self.create_subscription(
                Image,
                f'/tof_{name}/depth',
                lambda msg, name=name: self.depth_callback(msg, name),
                depth_qos
            )
            self.depth_subs.append(sub)
            
            # Create scan publisher
            pub = self.create_publisher(
                LaserScan,
                f'/scan/{name}',
                10
            )
            self.scan_pubs.append(pub)
            
        self.get_logger().info('ToF to LaserScan converter initialized')
    
    def depth_callback(self, msg: Image, sensor_name: str) -> None:
        """Convert depth image to LaserScan."""
        try:
            # Convert depth image to numpy array
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg)
            
            # Get sensor index
            sensor_idx = self.sensor_names.index(sensor_name)
            
            # Calculate sensor yaw based on mounting position
            sensor_yaw = sensor_idx * (math.pi / 4.0)  # 45° intervals
            
            # Collapse vertical columns to single range value
            # We take the minimum valid range from each column
            ranges = []
            for col in range(8):
                col_depths = depth_image[:, col]
                valid_depths = col_depths[
                    (col_depths >= self.range_min) & 
                    (col_depths <= self.range_max)
                ]
                if len(valid_depths) > 0:
                    ranges.append(float(np.min(valid_depths)))
                else:
                    ranges.append(float('inf'))
            
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header = msg.header
            scan_msg.angle_min = -self.h_fov / 2.0
            scan_msg.angle_max = self.h_fov / 2.0
            scan_msg.angle_increment = self.h_fov / 8.0
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / 30.0  # 30 Hz update rate
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            scan_msg.ranges = ranges
            
            # Publish scan
            self.scan_pubs[sensor_idx].publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ToF8x8ToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
