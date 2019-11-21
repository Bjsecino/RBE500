#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import numpy as np
import math


def dh_to_trans_mat(theta, d, a, alpha):
    mat = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                   [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                   [0, math.sin(alpha), math.cos(alpha), d],
                   [0, 0, 0, 1]])
    return mat


def callback(data):
    q1 = data.x
    q2 = data.y
    q3 = data.z

    print "q1 = %s rad" % q1
    print "q2 = %s rad" % q2
    print "q3 = %s meters" % q3

    d1 = 0.2
    a1 = 0.2
    a2 = 0.2
    d3 = 0.1 + q3

    T01 = dh_to_trans_mat(q1, d1, a1, 0)
    T12 = dh_to_trans_mat(q2, 0, a2, np.pi)
    T23 = dh_to_trans_mat(0, d3, 0, 0)

    T02 = T01.dot(T12)
    T03 = T02.dot(T23)

    print T03.round(2)


def forward_kin():

    rospy.init_node('forward_kin', anonymous=True)

    rospy.Subscriber("joints", Vector3, callback)

    rospy.spin()


if __name__ == '__main__':
    print "Starting forward_kin"
    forward_kin()



