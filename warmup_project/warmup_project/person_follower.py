import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PersonFollowerNode(Node):

    def __init__(self):
        super().__init__('person_follower_node')
        timer_period = 0.1
        self.velocity = 0.1
        self.angle_to_wall = 0

        self.data = []

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.create_subscription(LaserScan, 'projected_stable_scan', self.get_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_scan(self, scan_msg):
        # front_mean = np.mean(scan_msg.ranges[270:280])
        self.data = scan_msg.data

        


    def run_loop(self):
        msg = Twist()
        print(self.data)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
