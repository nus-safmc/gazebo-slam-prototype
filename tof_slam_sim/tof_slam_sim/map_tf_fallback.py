#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class MapTFFallback(Node):
    def __init__(self):
        super().__init__('map_tf_fallback')
        self.declare_parameter('parent_frame', 'robot/map')
        self.declare_parameter('child_frame', 'robot/odom')
        self.parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child = self.get_parameter('child_frame').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self, spin_thread=False)
        self.timer = self.create_timer(0.1, self._tick)
        self.had_real = False
        self.get_logger().info(f'Waiting for real TF {self.parent} -> {self.child}; publishing identity fallback at 10.0 Hz until then.')

    async def _try_lookup(self):
        try:
            return await self.buf.lookup_transform_async(self.parent, self.child, rclpy.time.Time())
        except Exception:
            return None

    def _tick(self):
        if self.had_real:
            return
        # we just publish identity; rviz/slam will switch to real TF once available
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = MapTFFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
