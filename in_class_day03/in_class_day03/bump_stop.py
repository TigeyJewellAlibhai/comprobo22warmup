#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump

class BumpStopNode(Node):

	def __init__(self):
		super().__init__('bump_stop')
		timer_period = 0.1
		self.velocity = 0.1
		self.bumper_active = False
		self.timer = self.create_timer(timer_period, self.run_loop)
		self.create_subscription(Bump, 'bump', self.get_bump, 10)
		self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

	def get_bump(self, bump_msg):
		self.bumper_active = (bump_msg.left_front or bump_msg.left_side or bump_msg.right_front or bump_msg.right_side)
		print("bump =", self.bumper_active)

	def run_loop(self):
		msg = Twist()
		msg.linear.x = self.velocity if not self.bumper_active else 0.0
		self.vel_pub.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = BumpStopNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
