""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point


my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
my_point = Point(x=1.0, y=2.0, z=0.0)
my_point_stamped = PointStamped(header=my_header, point=my_point)
print(my_point_stamped)
geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1662424018, nanosec=755100091), frame_id='odom'), point=geometry_msgs.msg.Point(x=1.0, y=2.0, z=0.0))

my_point_stamped = PointStamped(header=Header(stamp=self.get_clock().now().to_msg(),
                                    frame_id="odom"),
                                    point=Point(x=1.0, y=2.0, z=0.0))
self.publisher = self.create_publisher(PointStamped, 'my_point', 10)
self.publisher.publish(my_point_stamped)



class SendMessageNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def run_loop(self):
        print('Hi from in_class_day02.')

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendMessageNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()

