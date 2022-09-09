""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


class SendMessageNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1 # FLAG: runs every 10ms 
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(PointStamped, 'my_point', 10)
       

    def run_loop(self):
        
        my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        point = Point(x=1.0, y=2.0, z=0.0)
        msg = PointStamped(header=my_header, point=point)
        print(msg)
        self.publisher.publish(msg)
        #print('Hi from in_class_day02.')


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendMessageNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()

