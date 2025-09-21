#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutoPilot(Node):
    def __init__(self):
        super().__init__('auto_pilot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Tunables via env vars (no code changes needed)
        lin_x = float(os.environ.get('AP_LIN_X', '0.8'))   # body-x forward
        lin_y = float(os.environ.get('AP_LIN_Y', '0.0'))   # body-y
        lin_z = float(os.environ.get('AP_LIN_Z', '0.6'))   # body-z up (fight gravity)
        ang_z = float(os.environ.get('AP_ANG_Z', '0.2'))   # yaw rate

        self.msg = Twist()
        self.msg.linear.x  = lin_x
        self.msg.linear.y  = lin_y
        self.msg.linear.z  = lin_z
        self.msg.angular.z = ang_z

        hz = float(os.environ.get('AP_RATE', '10.0'))      # Hz
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'AutoPilot publishing @ {hz} Hz: lin=({lin_x},{lin_y},{lin_z}) ang.z={ang_z}'
        )

    def _tick(self):
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = AutoPilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

