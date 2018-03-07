#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


class nodeTest(object):
    def __init__(self):
        self.sub = rospy.Subscriber("test", Float64, self.callback)

    def callback(self, data):
        print "I am in callback ha ha rida"



class realNode(object):
    def __init__(self):
        rospy.init_node('super_master_node')
        self.sub = rospy.Publisher("tda", Float64)

        self.test = nodeTest()


if __name__ == '__main__':
    node = realNode()
    rospy.spin()
