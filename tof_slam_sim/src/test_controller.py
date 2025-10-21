#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

msg = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s : force stop
CTRL-C to quit
"""

moveBindings = {
    'w': (0.2, 0.0),
    'x': (-0.2, 0.0),
    'a': (0.0, 0.2),
    'd': (0.0, -0.2),
    'q': (0.2, 0.2),
    'e': (0.2, -0.2),
    'z': (-0.2, 0.2),
    'c': (-0.2, -0.2),
    's': (0.0, 0.0),
}

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        try:
            print(msg)
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    linear, angular = moveBindings[key]
                    twist = Twist()
                    twist.linear.x = linear
                    twist.angular.z = angular
                    self.publisher.publish(twist)
                elif key == '\x03':  # CTRL-C
                    break
                
        except Exception as e:
            print(e)
            
        finally:
            # Stop the robot
            twist = Twist()
            self.publisher.publish(twist)
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
