#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np


class LidarNode(object):

    def __init__(self):

        rospy.init_node('lidar_node')
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.get_distance)
        self.pub_dist = rospy.Publisher('lidar/front_distance', Float64, queue_size=10)

        # self.n = 10
        self.angle = 20.0

    def get_distance(self, data):

        ranges = np.array(data.ranges, np.float32)
        center = ranges.size // 2
        degree_res = int(self.angle * (ranges.size / 360.0))
        ranges = ranges[center - degree_res: center + degree_res]

        ranges = ranges[ranges < 40]
        if ranges.size == 0:
            return
        avg = np.average(ranges)
        # x = np.cumsum(ranges, dtype=float)
        #
        # for i in range(self.n, len(x)):
        #     x[i] = (x[i] - x[i - self.n]) / self.n
        #
        # x = x[self.n - 1:]
        #
        # min_dist = np.argmin(x)

        self.pub_dist.publish(avg)


if __name__ == '__main__':
    lidar_node = LidarNode()
    rospy.spin()
