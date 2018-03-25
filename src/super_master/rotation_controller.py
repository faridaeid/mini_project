#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32MultiArray, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class RotationController(object):
    def __init__(self):

        self.sub_euler = rospy.Subscriber("imu/data_euler", Vector3, self.set_robot_yaw)
        self.pub_motor = rospy.Publisher("motor/speed_motors", Int32MultiArray)    #TODO multiple instance to same publisher in same node

        self.rotation_angle = 0.0
        self.rotation_direction = 1
        self.rotation_speed = 50
        self.robot_yaw = 0.0
        self.start_yaw = 0.0
        self.end_yaw = 0.0

        self.motor_speed = Int32MultiArray()

    def adjust_degrees(self, deg):
        return (deg + 180) % 360

    def set_rotation_angle(self, angle):
        self.rotation_angle = angle

        if self.rotation_angle > 0:
            self.rotation_direction = -1
        else:
            self.rotation_direction = 1

    def set_robot_yaw(self, data):
        self.robot_yaw = self.adjust_degrees(data.z)

    def rotate(self):

        self.start_yaw = self.robot_yaw

        self.end_yaw = (self.start_yaw - self.rotation_angle) % 360

        # print 'rotation:', self.robot_yaw, self.end_yaw


        self.motor_speed.data = ([self.rotation_speed * self.rotation_direction,
                              self.rotation_speed * self.rotation_direction * -1])

        self.pub_motor.publish(self.motor_speed)

        while self.should_stop() != True:
            pass

        self.stop_rotation()

    def stop_rotation(self):

        self.motor_speed.data = ([0, 0])
        self.pub_motor.publish(self.motor_speed)

    def should_stop(self):
        # print self.robot_yaw, self.end_yaw, 'rotation:', self.start_yaw, self.end_yaw
        if abs(self.robot_yaw - self.end_yaw) < 1:
            return True
        else:
            # print self.robot_yaw, self.end_yaw
            return False

if __name__ == '__main__':
    rotate_node = RotationController()
