""" Explore publishing in ROS2 """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class SendMessageNode(Node):

    def __init__(self):
        super().__init__('send_message_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, 'my_marker', 10)

    def run_loop(self):
        """ 
        Creates a marker at (1.0, 2.0)
        """
        msg = Marker()
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose = Pose()
        msg.header.frame_id = "/odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'my_namespace'
        msg.id = 0
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 0.5
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SendMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
