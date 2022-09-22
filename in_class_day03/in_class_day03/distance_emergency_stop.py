#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# from neato2_interfaces.msg import Bump

class DistanceStopNode(Node):

	def __init__(self):
		super().__init__('bump_stop')
		timer_period = 0.1
		self.velocity = 0.1
		self.scan_active = False

		self.timer = self.create_timer(timer_period, self.run_loop)
		self.create_subscription(LaserScan, 'scan', self.get_scan, 10)
		self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

	def get_scan(self, scan_msg):
		# flag if there's an object in front of the robot within 0.5 meters, within the viewing range of -45 degrees to 45 degrees
		self.scan_active = max( [(i != 0 and i < 0.5) for i in scan_msg.ranges[0:45]] ) or max( [(i != 0 and i < 0.5) for i in scan_msg.ranges[315:360]] )
		print("self.scan_active =", self.scan_active)

	def run_loop(self):
		"""
		Drives neato forward, stops when there's an object within 0.5 meters directly in front.
		"""
		msg = Twist()
		msg.linear.x = self.velocity if not self.scan_active else 0.0
		self.vel_pub.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = DistanceStopNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
