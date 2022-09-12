
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveSquareNode(Node):

    def __init__(self):
        super().__init__('drive_square_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.loops_to_drive = 20
        self.loop = 0
        self.lin_vel = 0.3
        self.ang_vel = 0.8
        self.drive_mode = 0

    def run_loop(self):
        print("drive_mode", self.drive_mode)
        print("loop", self.loop)

        msg = Twist()
        if self.drive_mode == 0: # drive fwd
            # print("if 0")
            msg.linear.x = self.lin_vel 
            msg.angular.z = 0.0
        elif self.drive_mode == 1: # turn
            # print("if 1")
            msg.linear.x = 0.0 
            msg.angular.z = self.ang_vel
        else: # self.drive_mode == 2 means stopped
            # print("if 2")
            msg.linear.x = 0.0 
            msg.angular.z = 0.0
        print("msg", msg)

        self.publisher.publish(msg)

        self.loop += 1
        if self.loop == self.loops_to_drive:
            self.loop = 0
            self.drive_mode = (self.drive_mode + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
