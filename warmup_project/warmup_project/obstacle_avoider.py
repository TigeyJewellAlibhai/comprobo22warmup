from cmath import infj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math

class ObstacleAvoiderNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoider_node')
        timer_period = 0.1
        self.velocity = 0.1 # linear velocity to drive forward at by default
        self.angle = 0.1 # the angle of the object to avoid
        self.range = 0 # the range of the object to avoid

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.create_subscription(LaserScan, 'stable_scan', self.get_scan, 10)
        self.create_subscription(Odometry, 'odom', self.get_odom, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_angle = 0.1 # angle between current heading and original heading in the odom frame
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
                
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
            if -50 < avg[0] < 50 and avg[1] < min_range:
                min_range = avg[1]
                angle = avg[0]

        self.range = min_range
        self.angle = angle
        self.cluster_pos = self.pol2cart(min_range, angle)
        #print("min_range, angle, cluster_pos", min_range, angle, self.cluster_pos)

    def get_odom(self, odom_msg):
        """
        Gets "current_angle", the angle between the neato's current heading 
        and its' original heading of 0 degrees in the odom frame.
        """
        odom_pos = odom_msg.pose.pose.position
        odom_ori = self.euler_from_quaternion(odom_msg.pose.pose.orientation)[2]
        self.current_angle = odom_ori - 2*math.pi if odom_ori > math.pi else odom_ori

    def run_loop(self):
        """
        Drives the neato to avoid obstacles by turning away from the nearest obstacle,
        or drives straight if there is no nearby obstacle. 
        """

        msg = Twist()
        # If no obstacle in range, revert to odometry heading
        if self.range > 1:
            msg.angular.z = ((-self.current_angle/abs(self.current_angle))*0.8*(self.current_angle**2))
        # Else avoid the obstacle
        else:
            msg.angular.z = (10/-self.angle)
        msg.linear.x = 0.3
        print(self.current_angle)

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

