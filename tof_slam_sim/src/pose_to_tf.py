#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTf(Node):
    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')

        # The parent frame (e.g., 'map' or 'odom')
        self.parent_frame = self.declare_parameter(
            'parent_frame', 'map'
        ).get_parameter_value().string_value

        # The child frame (e.g., 'base_link')
        self.child_frame = self.declare_parameter(
            'child_frame', 'base_link'
        ).get_parameter_value().string_value

        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the PoseStamped topic from the bridge
        self.subscription = self.create_subscription(
            PoseStamped,
            '/model/x500_small_tof_0/pose',  # <-- Make sure this matches your bridge topic
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        t = TransformStamped()

        # Read info from the message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Copy the pose data
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
