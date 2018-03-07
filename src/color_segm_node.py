#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ColorSegNode:
    def __init__(self):
        rospy.init_node('color_segm_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("csi_cam/image_raw", Image, self.segment_image_callback)
        self.image_pub = rospy.Publisher("image_seg", Image, queue_size=2)
        self.image_pub_seg = rospy.Publisher("image_seg/seg", Image, queue_size=2)
        self.image_pub_cropped_img = rospy.Publisher("image_seg/cropped", Image, queue_size=2)
        self.image_pub_canny_img = rospy.Publisher("image_seg/canny_debug", Image, queue_size=2)
        self.target_center_pub = rospy.Publisher("target_center_x", Int32, queue_size=3)

    def _color_segmentation(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #        seg_image = cv2.inRange(hsv, np.array([24, 159, 113]), np.array([55, 247, 190]))

        seg_image = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        seg_image = cv2.medianBlur(seg_image, 11)
        # self.image_pub_seg.publish(self.bridge.cv2_to_imgmsg(seg_image, "passthrough"))

        #        kernel = np.ones((7, 7), np.uint8)
        #        seg_image = cv2.dilate(seg_image, kernel, iterations=1)
        #        seg_image = cv2.medianBlur(seg_image, 11)



        return seg_image

    def detect_arrow_direction(self):

        # imgray = cv2.cvtColor(self.cropped_image, cv2.COLOR_BGR2GRAY)
        # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(self.cropped_image, contours, -1, (0, 255, 0), 3)

        # threshold = self.cropped_image.shape[0] / 2 / 2
        threshold = 20
        # cropped_image_gray = cv2.cvtColor(self.cropped_image, cv2.COLOR_BGR2GRAY)
        # cropped_image_gray = cv2.blur(cropped_image_gray, (11,11))
        edges = cv2.Canny(self.seg_image_cropped, 50, 150, apertureSize=3)
        self.image_pub_canny_img.publish(self.bridge.cv2_to_imgmsg(edges, "passthrough"))
        # edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLines(edges, 0.7, np.pi/180 , threshold)

        lines = np.concatenate(lines, axis=0)

        diagonal1 = []
        diagonal2 = []

        # print lines.shape[0]
        # print lines.reshape(lines.shape[0], 0, 2)
        for rho, theta in lines:

            degree_theta = theta * 180 / np.pi

            if 10 < degree_theta < 80:
                diagonal1.append(np.array([theta, rho]))
            elif 120 < degree_theta < 170:
                diagonal2.append(np.array([theta, rho]))
            else:
                continue

        [theta1, rho1] = np.average(np.array(diagonal1), axis=0)
        [theta2, rho2] = np.average(np.array(diagonal2), axis=0)

        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]])

        b = np.array([[rho1], [rho2]])

        x0, y0 = np.linalg.solve(A, b)
        x0, y0 = int(np.round(x0)), int(np.round(y0))

        cv2.circle(self.cropped_image, (x0, y0), 2, (0,0,255), 10)

        center = self.cropped_image.shape[1] / 2

        if x0 > center:
            print "right"
        else:
            print "left"

        # hough lines
        # eleminate the 90s ish
        # get intersection
        # get center
        # condition between instersection and center

        # pass

    def _get_boundaries(self, seg_image, cv_image):

        output = cv2.connectedComponentsWithStats(seg_image, 4, cv2.CV_8U)
        stats = output[2]

        indx = 0
        if stats.shape[0] > 1:
            indx = np.argmax(stats[1:, 4]) + 1

        x, y, w, h, area = stats[indx]
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), thickness=2, lineType=8, shift=0)
        center_x = x + w // 2
        self.seg_image_cropped = seg_image[y:y+h, x:x+w]
        self.cropped_image = cv_image[y:y+h, x:x+w]
        self.cnt_x = center_x
        return center_x, cv_image

    def segment_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        seg_image = self._color_segmentation(cv_image)

        center_x, cv_image = self._get_boundaries(seg_image, cv_image)

        self.target_center_pub.publish(center_x)

        self.detect_arrow_direction()

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.image_pub_cropped_img.publish(self.bridge.cv2_to_imgmsg(self.cropped_image, "bgr8"))
            self.image_pub_seg.publish(self.bridge.cv2_to_imgmsg(self.seg_image_cropped, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    obj = ColorSegNode()
    rospy.spin()