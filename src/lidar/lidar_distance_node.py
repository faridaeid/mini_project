#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarDistanceNode(object):

    def __init__(self):

        rospy.init_node('lidar_distance_node')

        self.pub_distance = rospy.Publisher('lidar/closest_distance', Float64)
        self.sub = rospy.Subscriber("scan", LaserScan, self.get_min_distance)

    def get_min_distance(self, data):

        ret_val = self.min_dist(data.ranges)

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(ret_val))
        self.pub_distance.publish(ret_val)

    def min_dist(self, ranges, n=10, until=0.5):
        ranges = np.array(ranges, np.float32)
        ranges = ranges[2093:4186]
        ranges = ranges[ranges < 1E308]
        x = np.cumsum(ranges, dtype=float)

        for i in range(n, len(x)):
            x[i] = (x[i] - x[i - n]) / n

        x = x[n - 1:]

        min_dist = np.argmin(x)

        return x[min_dist]

if __name__ == '__main__':

    lidar = LidarDistanceNode()
    rospy.spin()


