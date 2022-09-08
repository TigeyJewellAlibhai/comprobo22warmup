#!/usr/bin/env python3

import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):

    def __init__(self):
        super().__init__('send_message_node')
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


    def run_loop(self):
        self.getKey()
        print(self.key)
        msg = Twist()
        if self.key == '\x03':
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            rclpy.shutdown()
        else:
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)

# settings = termios.tcgetattr(sys.stdin)
# key = None
# while key != '\x03':
#     key = getKey()
#     print(key)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
