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
    Finish = 5

class SuperMasterNode(object):

    def __init__(self):

        rospy.init_node('super_master_node')

        self.sub_lidar_front_dist = rospy.Subscriber('lidar/front_distance', Float64, self.set_close_dist)

        self.rotation_controller = RotationController()
        self.color_segmentation = ColorSegmentation()
        self.pid_controller = PidController()
        self.rotate_controller = RotationController()

        self.current_state = State.MovePID

        self.lidar_front = 0
        self.recovery_counter = 0

    def start_main_controller(self):

        rospy.sleep(5)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            if self.current_state == State.MovePID:
                # print "move pid"
                self.move_pid()
            elif self.current_state == State.StopPID:
                # print "stop"
                self.stop_pid()
            elif self.current_state == State.DetectSign:
                # print "detect_sign"
                self.detect_sign()
            elif self.current_state == State.Rotate:
                # print "rotate"
                self.rotate()
            elif self.current_state == State.Finish:
                self.finish()

            rate.sleep()

    def move_pid(self):

        center_x = self.color_segmentation.get_center_target()
        area = self.color_segmentation.get_target_area()

        # print "Lidar Front", self.lidar_front
        print "move pid ", "lidar front ", self.lidar_front, "area: ", area
        # if self.lidar_front < 0.7:
        if area < 200 or area > 30000:
            self.pid_controller.idle()

            if self.recovery_counter > 4:
                self.recovery_counter = 0
                self.rotation_controller.set_rotation_angle(180)
                self.current_state = State.Rotate
            else:
                self.recovery_counter += 1

        # elif area > 16500:
        # elif area > 15500:
        elif area > 14500:

                self.current_state = State.StopPID

        else:
            self.recovery_counter = 0
            self.pid_controller.start_pid(center_x)

    def stop_pid(self):

        # if self.lidar_front == 0 or self.lidar_front > 0.6:
        #     self.current_state = State.MovePID
        # else:
        self.pid_controller.stop_pid()
        self.current_state = State.DetectSign

    def detect_sign(self):

        sign_detected = self.color_segmentation.detect_sign()
        print sign_detected

        if sign_detected == "circle":
            print "circle"
            self.current_state = State.Finish
        else:
            if sign_detected == "right" or sign_detected == "left":
                if sign_detected == "right":
                    self.rotation_controller.set_rotation_angle(90)
                else:
                    self.rotation_controller.set_rotation_angle(-90)

                self.current_state = State.Rotate

    def rotate(self):
        self.rotation_controller.rotate()
        self.current_state = State.MovePID

    def set_close_dist(self, data):
        self.lidar_front = data.data

    def finish(self):
        self.pid_controller.idle()
if __name__ == '__main__':
    mastar_node = SuperMasterNode()
    mastar_node.start_main_controller()