#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32, Int32MultiArray

from pid import PID


class PidController:
    def __init__(self):

        self.speed_pub = rospy.Publisher("motor/speed_motors", Int32MultiArray)

        self.pid = PID(0.3, 0.0, 0.0)
        self.pid.set_setpoint(150)

    def start_pid(self, center_x):

        pid_output = self.pid.update(center_x)

        constant_speed = 40

        left_motor_speed = constant_speed - pid_output
        right_motor_speed = constant_speed + pid_output

        speeds = Int32MultiArray()
        speeds.data = [right_motor_speed, left_motor_speed]

        self.speed_pub.publish(speeds)

    def stop_pid(self):

        speeds = Int32MultiArray()
        speeds.data = [0, 0]
        self.speed_pub.publish(speeds)


if __name__ == '__main__':
    obj = PidController()
