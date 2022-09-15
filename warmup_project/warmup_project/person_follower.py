from cmath import infj
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
        self.data = 0
        self.size = []

        

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.create_subscription(LaserScan, 'stable_scan', self.get_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def cluster(data):
        clusters = []
        clusters.append[(0, data[0])]
        for point in data:
            data_tuple = (data.index(point), point)
            for cluster in clusters:
                for check in cluster:
                    if(min(abs(check[0]-data_tuple[0]), abs(check[0]-data_tuple) - 360) < 5 and abs(check[1]-data_tuple[1]) < 0.5):
                        cluster.append(data_tuple)
                        break
                else:
                    continue
                break

        cluster_avg = []
        for cluster in clusters:
            cluster_avg.append((np.mean(cluster[:][0]), np.mean(cluster[:][1])))
        return [clusters, cluster_avg]
    
    def cluster2(self, data):
        clusters = []
        in_cluster = False
        for i,point in enumerate(data):
            if point not in [0, float('inf')]:
                if not in_cluster:
                    clusters.append([[data.index(point), point]])
                else:
                    clusters[-1].append([data.index(point), point])
                in_cluster = True
            else:
                in_cluster = False
        
        avgs = []
        for cluster in clusters:
            avgs.append([np.mean(x) for x in zip(*cluster)])
        return clusters, avgs

                
    def get_scan(self, scan_msg):
        data = scan_msg.ranges
        clusters, avgs = self.cluster2(data)
        target = np.array(avgs)
        target = np.where(avgs == np.min(avgs))
        print(target)

    def run_loop(self):
        print(self.size)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
