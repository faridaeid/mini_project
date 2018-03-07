#!/usr/bin/env python


import rospy
import numpy as np
from pid import PID
from std_msgs.msg import Int32, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class PidToSpeedNode:
    def __init__(self):
        rospy.init_node('pid_speed_node')

        self.sub = rospy.Subscriber('target_center_x', Int32, self.change_pid_to_speed)
        self.speed_pub = rospy.Publisher("speed_topic", Int32MultiArray)

        self.pid = PID(0.3, 0.0, 0.0)
        self.pid.set_setpoint(150)


    def change_pid_to_speed(self, data):

        pid_output = self.pid.update(data.data)

        constant_speed = 30

        left_motor_speed = constant_speed - pid_output
        right_motor_speed = constant_speed + pid_output

        print "pid_output", pid_output, " Left Speed = " , left_motor_speed, "\tRight Speed", right_motor_speed

        speeds = Int32MultiArray()
        # speeds.layout.dim = 3
        # speeds.layout.dim[0].size = 1
        # speeds.layout.dim[1].size = 2
        speeds.data = [right_motor_speed, left_motor_speed]

        self.speed_pub.publish(speeds)

if __name__ == '__main__':
    obj = PidToSpeedNode()
    rospy.spin()
