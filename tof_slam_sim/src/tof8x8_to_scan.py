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
            # Convert depth image to numpy array. Prefer direct buffer parsing so we
            # consistently get floating-point meters regardless of encoding.
            depth_image = self._to_depth_array(msg)
            
            # Get sensor index
            sensor_idx = self.sensor_names.index(sensor_name)
            
            # Calculate sensor yaw based on mounting position
            sensor_yaw = sensor_idx * (math.pi / 4.0)  # 45° intervals
            
            # Collapse vertical columns to single range value
            # We take the minimum valid range from each column
            ranges = []
            columns = min(depth_image.shape[1], 8)
            for col in range(columns):
                col_depths = depth_image[:, col]
                valid_mask = (
                    np.isfinite(col_depths) &
                    (col_depths >= self.range_min) &
                    (col_depths <= self.range_max)
                )
                valid_depths = col_depths[valid_mask]
                if len(valid_depths) > 0:
                    ranges.append(float(np.min(valid_depths)))
                else:
                    ranges.append(float('inf'))
            while len(ranges) < 8:
                ranges.append(float('inf'))

            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header = msg.header
            scan_msg.header.frame_id = 'base_link'
            scan_msg.angle_min = -self.h_fov / 2.0
            scan_msg.angle_max = self.h_fov / 2.0
            if len(ranges) > 1:
                scan_msg.angle_increment = self.h_fov / (len(ranges) - 1)
            else:
                scan_msg.angle_increment = self.h_fov
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / 30.0  # 30 Hz update rate
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            scan_msg.ranges = ranges
            
            # Publish scan
            self.scan_pubs[sensor_idx].publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def _to_depth_array(self, msg: Image) -> np.ndarray:
        """Return depth array in meters regardless of encoding."""
        try:
            float_encodings = {
                '32FC1', '32FC', '32F', 'FLOAT32', 'float32',
                'R32F', 'R_FLOAT32'
            }
            if msg.encoding in float_encodings:
                return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            if msg.encoding == '64FC1':
                return np.frombuffer(msg.data, dtype=np.float64).reshape(msg.height, msg.width).astype(np.float32)

            # Fall back to cv_bridge for other encodings (e.g., rgb8 for debug visuals)
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_img.ndim == 3:
                # Use first channel; assume data encoded 0-1 or 0-255 range
                cv_img = cv_img[:, :, 0]
            return cv_img.astype(np.float32)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().error(f'Failed to decode depth image ({msg.encoding}): {exc}')
            return np.full((msg.height or 1, msg.width or 1), float('inf'), dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ToF8x8ToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
