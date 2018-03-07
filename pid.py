#!/usr/bin/env python

import rospy
from std.msg


class PID(object):

    def __init__(self, ):

        rospy.init_node('pid_node', anonymous=True)

        self.motor_pub = rospy.Publisher('motor_value', int)
        self.bound_subs = rospy.Subscriber('boundary_center', int, self.set_current_state)

        self.Kd = 0
        self.Kp = 0
        self.Ki = 0
        self.set_point = 320
        self.current_state = None

        self.last_input = 0
        self.output = 0

        self.motor_value_left = None
        self.motor_value_right = None

    def set_current_state(self, data):

        self.current_state = data.name  #TODO
        self.calc_error()
        self.publish_motor()

    def calc_error(self):

        error = self.current_state - self.set_point
        delta_error = self.current_state - self.last_input

        self.output = self.Kp * error + self.Ki - self.Kd * delta_error

    def publish_motor(self, value):

        self.motor_pub.publish(value)


if __name__ == '__main__':

    pid = PID()
    rospy.spin()
