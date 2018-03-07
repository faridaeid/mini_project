#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32MultiArray, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class RotateNode(object):
    def __init__(self):
        rospy.init_node('rotate_node')

        self.sub_euler = rospy.Subscriber("imu/data_euler", Vector3, self.set_robot_yaw)
        self.sub_order = rospy.Subscriber("rotate_now", Float64, self.set_rotation_angle)
        self.pub_motor = rospy.Publisher("speed_topic", Int32MultiArray)    #TODO change topic name and qsize
        self.pub_rotation_status = rospy.Publisher("rotation_status", Bool)     #TODO convention in topic names

        self.rotation_angle = 0.0
        self.rotation_direction = 1
        self.rotation_speed = 45
        self.robot_yaw = 0.0

    def adjust_degrees(self, deg):
        return (deg + 180) % 360

    def set_rotation_angle(self, data):
        self.rotation_angle = data.data

        if self.rotation_angle > 0:
            self.rotation_direction = -1
        else:
            self.rotation_direction = 1

        self.rotate()

    def set_robot_yaw(self, data):
        self.robot_yaw = self.adjust_degrees(data.z)

    def rotate(self):

        starting_yaw = self.robot_yaw

        rotation_values = Int32MultiArray()
        rotation_values.data = ([self.rotation_speed * self.rotation_direction,
                              self.rotation_speed * self.rotation_direction *- 1])
        self.pub_motor.publish(rotation_values)

        while abs(starting_yaw - self.robot_yaw) < (self.rotation_angle):
            print abs(starting_yaw - self.robot_yaw), starting_yaw, self.robot_yaw, self.rotation_angle
            continue
        print abs(starting_yaw - self.robot_yaw), starting_yaw, self.robot_yaw, self.rotation_angle
        self.stop_rotation()

    def stop_rotation(self):

        self.rotation_angle = 0.0

        brake_values = Int32MultiArray()
        brake_values.data = ([0, 0])
        self.pub_motor.publish(brake_values)
        print "stopping rotation"
        self.pub_rotation_status.publish(True)


if __name__ == '__main__':
    rotate_node = RotateNode()
    rospy.spin()
