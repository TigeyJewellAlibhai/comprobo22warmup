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
        self.key_mapping = {'q' : (1.0,0.0,0.0,0.0,0.0,1.0), 
                            'w' : (1.0,0.0,0.0,0.0,0.0,0.0), 
                            'e' : (1.0,0.0,0.0,0.0,0.0,-1.0), 
                            'a' : (0.0,0.0,0.0,0.0,0.0,1.0), 
                            's' : (0.0,0.0,0.0,0.0,0.0,0.0), 
                            'd' : (0.0,0.0,0.0,0.0,0.0,-1.0), 
                            'z' : (-1.0,0.0,0.0,0.0,0.0,-1.0), 
                            'x' : (-1.0,0.0,0.0,0.0,0.0,0.0), 
                            'c' : (-1.0,0.0,0.0,0.0,0.0,1.0) 
                            }
        self.vel_coeff = 0.5
        #self.prev_key = self.key

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

        # else:
        #if self.key not in self.key_mapping or self.key == self.prev_key:
        #    self.key = 's'
        speeds = self.key_mapping[self.key]
        msg.linear.x = speeds[0] * self.vel_coeff 
        msg.linear.y = speeds[1] * self.vel_coeff 
        msg.linear.z = speeds[2] * self.vel_coeff 
        msg.angular.x = speeds[3] * self.vel_coeff 
        msg.angular.y = speeds[4] * self.vel_coeff 
        msg.angular.z = speeds[5] * self.vel_coeff 
        
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
