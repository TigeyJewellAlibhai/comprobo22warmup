import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower_node')
        timer_period = 0.1
        self.velocity = 0.1 # forward constant velocity
        self.angle_to_wall = 0 # the angle the neato is facing relative to the wall

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.get_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_scan(self, scan_msg):
        """ 
        Locate points directly to the right of the robot and calculate a 
        metric of how the neato is angled in relation to the wall.
        """
        sum_front, count_front = 0, 0
        for i in scan_msg.ranges[270:280]:
            if (i != 0 and i < 1.7):
                sum_front += i
                count_front += 1

        if count_front == 0:
            return "Error: couldn't find wall in front on right side"
        front = sum_front / count_front

        sum_behind, count_behind = 0, 0
        for i in scan_msg.ranges[260:270]:
            if (i != 0 and i < 1.7):
                sum_behind += i
                count_behind += 1
        if count_behind == 0:
            return "Error: couldn't find wall behind on right side"
        behind = sum_behind / count_behind

        self.angle_to_wall = (behind - front)


    def run_loop(self):
        """ 
        Drive robot to follow wall on its righthand side. 
        If no wall is seen, it drives forward at a constant speed. 
        """
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = float(min(self.angle_to_wall * 4, 1.0))
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
