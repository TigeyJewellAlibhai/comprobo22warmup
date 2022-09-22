import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveSquareNode(Node):

    def __init__(self):
        super().__init__('drive_square_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.loops_to_drive = 20 # the number of loops to drive forward 1 meter or to turn 90 degrees
        self.loop = 0 # the current loop count (each loop being 0.1 seconds from timer_period)
        self.lin_vel = 0.3 # the linear vel to drive fwd at
        self.ang_vel = 0.8 # the angular vel to turn at
        self.drive_mode = 0 # drive mode. 0 = fwd, 1 = turn, 2 = stop

    def run_loop(self):
        """ 
        Drives robot in a square using a count of the number of times run_loop has been called. 
        """
        msg = Twist()
        if self.drive_mode == 0: # drive fwd
            msg.linear.x = self.lin_vel 
            msg.angular.z = 0.0
        elif self.drive_mode == 1: # turn
            msg.linear.x = 0.0 
            msg.angular.z = self.ang_vel
        else: # self.drive_mode == 2 means stopped
            msg.linear.x = 0.0 
            msg.angular.z = 0.0

        self.publisher.publish(msg)

        self.loop += 1 # loop iterator, constantly increasing
        if self.loop >= self.loops_to_drive * 8: # after 4 turns + 4 straights, the robot has completed a square
            msg = Twist()
            self.publisher.publish(msg) # make the robot stop driving. 
            rclpy.shutdown()

        elif (self.loop % self.loops_to_drive) == self.loops_to_drive - 1: # increment drive mode every time it passes 20 more loops 
            self.drive_mode = (self.drive_mode + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
