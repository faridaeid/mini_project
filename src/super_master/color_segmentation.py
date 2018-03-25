#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ColorSegmentation:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("csi_cam/image_raw", Image, self.segment_image)
        self.image_bound = rospy.Publisher("cam/image_raw", Image)
        self.image_bound_canny = rospy.Publisher("cam/image_canny", Image)
        self.image_bound_crooped = rospy.Publisher("cam/image_cropped", Image)
        self.image_bound_seg = rospy.Publisher("cam/image_seg", Image)

        self.image_center_x = 0
        self.target_area = 0
        self.seg_image_cropped = None

        self.threshold = 20

        #HSV for neon yellow [20, 100, 100] [30, 255, ,255]
        #HSV for hot pink [150, 100, 100] [180, 255, 255]
        #HSV GREEN [70, 210, 90 ][80, 255, 120]
        # seg_image = cv2.inRange(hsv, np.array([50, 120, 50]), np.array([80, 255, 255]))


    def _color_segmentation(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        seg_image = cv2.inRange(hsv, np.array([50, 120, 50]), np.array([80, 255, 255]))
        # seg_image = cv2.medianBlur(seg_image, 11)

        return seg_image

    def _detect_circle(self, greyimg):

        greyimg = cv2.medianBlur(greyimg, 5)
        circles = cv2.HoughCircles(greyimg, cv2.HOUGH_GRADIENT, 1, 1, param1=30, param2=50, minRadius=5,
                                   maxRadius=140) ##TODO use radius to change circle parameter

        if circles is not None:
            print("detected circle: ", circles)
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # draw the outer circle
                print "radius", i[2] #TODO use radius to change circle parameter
                cv2.circle(self.cropped_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                # cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

                try:
                    self.image_bound_crooped.publish(self.bridge.cv2_to_imgmsg(self.cropped_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                return "circle"

        else:
            return "not found"



    def _detect_sign_inner(self):

        edges = cv2.Canny(self.seg_image_cropped, 50, 150, apertureSize=3)
        # self.image_pub_canny_img.publish(self.bridge.cv2_to_imgmsg(edges, "passthrough"))
        #

        grey_img = cv2.cvtColor(self.cropped_image, cv2.COLOR_BGR2GRAY)

        # cv2.imwrite("/home/nvidia/gery.jpg", grey_img)
        circle_result = self._detect_circle(grey_img)

        if circle_result == "circle":
            return circle_result

        lines = cv2.HoughLines(edges, 0.9, np.pi / 180, self.threshold)

        if lines is None:
            return "repeat"

        lines = np.concatenate(lines, axis=0)

        diagonal1 = []
        diagonal2 = []

        for rho, theta in lines:

            degree_theta = theta * 180 / np.pi

            if 10 < degree_theta < 80:
                diagonal1.append(np.array([theta, rho]))
            elif 120 < degree_theta < 170:
                diagonal2.append(np.array([theta, rho]))
            else:
                continue

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(self.cropped_image, (x1, y1), (x2, y2), (0, 0, 255), 2)


        if not len(diagonal2) or not len(diagonal1):
            return "repeat"

        [theta1, rho1] = np.average(np.array(diagonal1), axis=0)
        [theta2, rho2] = np.average(np.array(diagonal2), axis=0)

        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]])

        b = np.array([[rho1], [rho2]])

        x0, y0 = np.linalg.solve(A, b)
        x0, y0 = int(np.round(x0)), int(np.round(y0))

        cv2.circle(self.cropped_image, (x0, y0), 2, (0, 0, 255), 10)

        center = self.cropped_image.shape[1] / 2

        try:
            self.image_bound_crooped.publish(self.bridge.cv2_to_imgmsg(self.cropped_image, "bgr8"))
            # self.image_bound_canny.publish(self.bridge.cv2_to_imgmsg(self.image_bound_canny, "passthrough"))

        except CvBridgeError as e:
            print(e)

        if x0 > center:
            return "right"
        else:
            return "left"

    def detect_sign(self):

        direction = self._detect_sign_inner()

        while direction == "repeat":
            direction = self._detect_sign_inner()

        return direction

    def _get_boundaries(self, seg_image, cv_image):

        output = cv2.connectedComponentsWithStats(seg_image, 4, cv2.CV_8U)
        stats = output[2]

        indx = 0
        if stats.shape[0] > 1:
            indx = np.argmax(stats[1:, 4]) + 1  #TODO here

        x, y, w, h, area = stats[indx]
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), thickness=2, lineType=8, shift=0)
        center_x = x + w // 2
        center_y = y + h // 2

        # cv2.circle(cv_image, (center_x, center_y), 2, (0,0,255), 2)
        self.seg_image_cropped = seg_image[y:y+h, x:x+w]
        self.cropped_image = cv_image[y:y+h, x:x+w]
        self.image_center_x = center_x
        self.image_center_y = center_y

        # print center_x

        cv2.circle(cv_image, (center_x, center_y), 2,  (0,0,255), 2)
        cv2.circle(cv_image, (center_x, center_y), 2,  (0,0,255), 2)

        self.target_area = w*h

        try:

            self.image_bound.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.image_bound_crooped.publish(self.bridge.cv2_to_imgmsg(self.cropped_image, "bgr8"))
            self.image_bound_seg.publish(self.bridge.cv2_to_imgmsg(seg_image, "passthrough"))


        except CvBridgeError as e:
            print(e)

    def segment_image(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        seg_image = self._color_segmentation(cv_image)

        self._get_boundaries(seg_image, cv_image)

    def get_center_target(self):
        return self.image_center_x

    def get_target_area(self):
        return self.target_area


if __name__ == '__main__':
    obj = ColorSegmentation()
    rospy.spin()