from cmath import infj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import copysign
from geometry_msgs.msg import PointStamped, Point, Pose
from visualization_msgs.msg import Marker
import numpy as np
import math

class FiniteStateMachineNode(Node):

    def __init__(self):
        super().__init__('finite_state_machine_node')
        timer_period = 0.1
        self.angle = 0.1 # the angle of the cluster/person to follow
        self.range = 0.1 # the range of the cluster/person to follow
        self.cluster_pos = [0.0, 0.0] # default cluster position

        self.loops_to_drive = 20 # the number of loops to drive forward 1 meter or to turn 90 degrees
        self.loop = 0 # the current loop count (each loop being 0.1 seconds from timer_period)
        self.lin_vel = 0.3 # the linear vel to drive fwd at
        self.ang_vel = 0.8 # the angular vel to turn at
        self.drive_mode = 0 # drive mode. 0 = fwd, 1 = turn, 2 = stop

        self.behaviour_mode = 'drive_square' # describes which behaviour mode to follow, used to switch between behaviours

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.create_subscription(LaserScan, 'stable_scan', self.get_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher = self.create_publisher(Marker, 'my_marker', 10)
    
    def cluster2(self, data):
        """ 
        Clusters lidar scan data. 
        Returns the average range and average angle of each cluster identified. 
        """

        clusters = []

        # creates list of angles from 0 to 180 then -179 back down to -1 to map onto data instead of 0 to 360
        angles_translate = [i for i in range(0,181)] + [i for i in range(-179,0,1)]
        ranges_mapped = {}

        # map data from 0 to 360 angles to -180 to 180 angles instead
        for i in range(0,360):
            ranges_mapped[angles_translate[i]] = data[i]

        in_cluster = False
        for ang in sorted(ranges_mapped.keys()):
            if ranges_mapped[ang] not in [0, float('inf')]: # if is valid range value

                # too far from current cluster, create new cluser
                if not in_cluster or (ang > -179 and abs(ranges_mapped[ang-1]- ranges_mapped[ang]) > 0.3): 
                    clusters.append([[ang, ranges_mapped[ang]]])
                else: # if close enough to current cluster, add to cluster
                    clusters[-1].append([ang, ranges_mapped[ang]])
                in_cluster = True

            elif ang > -179: # stay in cluster even if there's one outlier
                if ranges_mapped[ang-1] not in [0, float('inf')]:
                    in_cluster = True

            else: # not valid range
                in_cluster = False
        
        # filter clusters on number of points and return the average angle and range
        avgs = []
        for cluster in clusters: 
            if len(cluster) > 5 and len(cluster) < 90:
                avgs.append([np.mean(x) for x in zip(*cluster)])
        return clusters, avgs

    def pol2cart(self, rho, phi):
        """ 
        Convert polar coordinate to cartesian. 
        Accepts rho (range in meters), phi (angle in degrees).
        Outputs (x, y) distance in meters.
        """
        x = rho * np.cos(math.radians(phi))
        y = rho * np.sin(math.radians(phi))
        return (x, y)
                
    def get_scan(self, scan_msg):
        """ 
        Gets the position, angle, and range of the nearest obstacle.
        """
        data = scan_msg.ranges
        clusters, avgs = self.cluster2(data)
        min_range = 100
        angle = 0
        for avg in avgs:
            if avg[1] < min_range:
                min_range = avg[1]
                angle = avg[0]

        self.range = min_range
        self.angle = angle
        self.cluster_pos = self.pol2cart(min_range, angle)
        # print("min_range, angle, cluster_pos", min_range, angle, self.cluster_pos)

        if self.range > 1.5: # if nearest cluster average range is farther than 2 meters
            self.behaviour_mode = 'drive_square' # drive square mode
        else:
            self.behaviour_mode = 'person_follower' # a person/cluster nearby; person follower mode
            self.loop = 0 # reset loop count 

    def run_drive_square(self):
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

        self.vel_pub.publish(msg)

        self.loop += 1

        if (self.loop % self.loops_to_drive) == self.loops_to_drive - 1:
            self.drive_mode = (self.drive_mode + 1) % 2

    def run_person_follower(self):
        """
        Drives the robot toward the nearest cluster/person, and
        publishes a marker on top of this cluster to visualize it.
        """

        # create marker to show the position of the cluster that the neato is driving towards
        msg = Marker()
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose = Pose()
        msg.header.frame_id = "/base_link"
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
        msg.pose.position.x = self.cluster_pos[0]-0.05 # offset due to lidar being backset from center of neato
        msg.pose.position.y = self.cluster_pos[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)

        msg = Twist()
        if self.angle != 0.0: # if robot is facing straight on to the target, adjust linear velocity
            msg.linear.x = (1.5 * (self.range - 0.5)/abs(self.range - 0.5) * (self.range-0.5)**2)/(abs(self.angle)**0.5)
        else: # if robot is at an angle to the target, adjust linear velocity
            msg.linear.x = (1.5 * (self.range - 0.5)/abs(self.range - 0.5) * (self.range-0.5)**2)
        
        msg.angular.z = (0.015*self.angle) # set angular velocity based on angle difference from the target
        self.vel_pub.publish(msg)

    def run_loop(self):
        """
        Switch between drive_square and person_follower mode based on "behaviour_mode".
        """
        if self.behaviour_mode == 'drive_square':
            self.run_drive_square()
        elif self.behaviour_mode == 'person_follower':
            self.run_person_follower()

        # print("self.loop, self.drive_mode", self.loop, self.drive_mode)


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
