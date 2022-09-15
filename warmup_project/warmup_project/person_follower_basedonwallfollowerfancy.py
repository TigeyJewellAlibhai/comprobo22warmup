""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

import math
import matplotlib.pyplot as plt


class WallApproachNode(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('wall_approach')

        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None

        # the following three lines are needed to support parameters
        self.declare_parameters(namespace='', parameters=[('Kp', 0.5), ('target_distance', 0.5)])
        self.Kp = self.get_parameter('Kp').value
        self.target_distance = self.get_parameter('target_distance').value

        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.prev_scan = None

        plt.ion()


    def parameter_callback(self, params):
        """ This callback allows the parameters of the node to be adjusted
            dynamically by other ROS nodes (e.g., dynamic_reconfigure).
            This function is only needed to support dynamic_reconfigure. """
        for param in params:
            if param.name == 'Kp' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
            elif param.name == 'target_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.target_distance = param.value
        print(self.Kp, self.target_distance)
        return SetParametersResult(successful=True)


    def run_loop(self):
        msg = Twist()
        if self.distance_to_obstacle is None:
            # if we haven't seen an obstacle yet, just go straight at fixed vel
            # msg.linear.x = 0.1
            msg.linear.x = 0.0
        else:
            # use proportional control to set the velocity
            msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)

        self.vel_pub.publish(msg)


    # LILO: primarily all we do to modify the fancy wall follower code is to edit process_scan
    def process_scan(self, msg):
        # print("len", len(msg.ranges[90:-90]))
        # print("0", (msg.ranges[0]))

        # Fields in LaserScan for reference:
        # msg.header           # timestamp in the header is the acquisition time of 
        # msg.angle_min        # start angle of the scan [rad]
        # msg.angle_max        # end angle of the scan [rad]
        # msg.angle_increment  # angular distance between measurements [rad]
        # msg.time_increment   # time between measurements [seconds] - if your scanner
        # msg.scan_time        # time between scans [seconds]
        # msg.range_min        # minimum range value [m]
        # msg.range_max        # maximum range value [m]
        # msg.ranges           # range data [m] (Note: values < range_min or > range_max should be discarded)
        # msg.intensities      # intensity data [device-specific units].  If your


        threshold = 0.1
        moved_pts_theta = []
        moved_pts_range = []

        if self.prev_scan != None:
            for i in range(-90,90):
                for j in range(-5,5+1):
                    if abs(msg.ranges[i] - self.prev_scan[i+j]) < threshold:
                        break
                    elif j == 5:
                        moved_pts_theta.append(math.radians(i))
                        moved_pts_range.append(msg.ranges[i])

        # plt.polar([i for i in range(-90,90)], msg.ranges[-90:90])
        plt.clf()
        plt.polar(moved_pts_theta, moved_pts_range, 'o')
        plt.pause(0.01)

        print("moved_pts", moved_pts_theta, moved_pts_range)

        self.prev_scan = msg.ranges


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
