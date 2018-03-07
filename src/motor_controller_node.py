#!/usr/bin/env python

import rospy
import numpy as np
from motor_controller import MotorController
from std_msgs.msg import Int32MultiArray


class MotorControllerNode(object):

    def __init__(self):

        rospy.init_node('motor_controller_node')
        self.motor = MotorController()
        sub = rospy.Subscriber('speed_topic', Int32MultiArray, self.adjust_motor)

    def adjust_motor(self, data):
        speed_values = np.array(data.data)
        right_speed = speed_values[0]
        left_speed = speed_values[1]

        self.motor.cmd_move_motors(right_speed, left_speed)

    def stop(self):
        self.motor.brake()


if __name__ == '__main__':
    motor_node = MotorControllerNode()
    while not rospy.is_shutdown():
        rospy.spin()

    motor_node.stop()

