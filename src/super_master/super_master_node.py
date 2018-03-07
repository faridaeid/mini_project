#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int32MultiArray
from rotation_controller import RotationController
from color_segmentation import ColorSegmentation
from pid_controller import PidController
from rotation_controller import RotationController

from enum import Enum

class State(Enum):
    MovePID = 1
    StopPID = 2
    Rotate = 3
    DetectSign = 4

class SuperMasterNode(object):

    def __init__(self):

        rospy.init_node('super_master_node')

        # self.sub_close_dist = rospy.Subscriber('lidar/closest_distance', Float64, self.set_close_dist)

        self.rotation_controller = RotationController()
        self.color_segmentation = ColorSegmentation()
        self.pid_controller = PidController()
        self.rotate_controller = RotationController()

        self.current_state = State.MovePID

    def start_main_controller(self):

        rospy.sleep(5)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.current_state == State.MovePID:
                self.move_pid()
            elif self.current_state == State.StopPID:
                self.stop_pid()
            elif self.current_state == State.DetectSign:
                print "detect sign"
                self.detect_sign()
            elif self.current_state == State.Rotate:
                self.rotate()
            rate.sleep()

    def move_pid(self):
        center_x = self.color_segmentation.get_center_target()
        area = self.color_segmentation.get_target_area()
        print area
        if area < 17000:
            self.pid_controller.start_pid(center_x)
        else:
            self.current_state = State.StopPID

    def stop_pid(self):
        self.pid_controller.stop_pid()
        self.current_state = State.DetectSign

    def detect_sign(self):

        sign_detected = self.color_segmentation.detect_arrow_direction()

        if sign_detected == "right" or sign_detected == "left":
            if sign_detected == "right":
                self.rotation_controller.set_rotation_angle(90)
            else:
                self.rotation_controller.set_rotation_angle(-90)

            self.current_state = State.Rotate

    def rotate(self):
        self.rotation_controller.rotate()
        self.current_state = State.MovePID

    def get_direction(self):
        direction = self.color_segmentation.detect_arrow_direction()


if __name__ == '__main__':
    mastar_node = SuperMasterNode()
    mastar_node.start_main_controller()