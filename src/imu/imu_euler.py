#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class ImuToEulerNode(object):
    def __init__(self):
        rospy.init_node('imu_euler_node')

        self.sub = rospy.Subscriber("imu/data", Imu, self.convert_to_euler)
        self.pub = rospy.Publisher("imu/data_euler", Vector3)

    def convert_to_euler(self, data):

        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0] * 180.0 / 3.14159265
        pitch = euler[1] * 180.0 / 3.14159265
        yaw = euler[2] * 180.0 / 3.14159265

        self.pub.publish(Vector3(roll, pitch, yaw))

if __name__ == '__main__':
    obj = ImuToEulerNode()
    rospy.spin()
