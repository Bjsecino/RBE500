#!/usr/bin/env python
import rospy
from rbe_500.srv import ForwardKin,ForwardKinResponse
from rbe_500.srv import Jacobian,JacobianResponse
from geometry_msgs.msg import Vector3
import numpy as np
import math


def dh_to_trans_mat(theta, d, a, alpha):
    mat = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                   [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                   [0, math.sin(alpha), math.cos(alpha), d],
                   [0, 0, 0, 1]])
    return mat

def joint_params_to_forward_kin(q1, q2, q3):
    d1 = 0.2
    a1 = 0.2
    a2 = 0.2
    d3 = 0.1 + q3

    T01 = dh_to_trans_mat(q1, d1, a1, 0)
    T12 = dh_to_trans_mat(q2, 0, a2, np.pi)
    T23 = dh_to_trans_mat(0, d3, 0, 0)

    T02 = T01.dot(T12)
    T03 = T02.dot(T23)
    return T03

def handle_forward_kin(req):
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    print "q1 = %s rad" % q1
    print "q2 = %s rad" % q2
    print "q3 = %s meters" % q3

    T03 = joint_params_to_forward_kin(q1, q2, q3)

    print T03.round(2)

    return str(T03.round(2))

def handle_jacobian(req):
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    print "q1 = %s rad" % q1
    print "q2 = %s rad" % q2
    print "q3 = %s meters" % q3

    J = np.array([[-0.2*np.sin(q1) - 0.2*np.sin(q1+q2), -0.2*np.sin(q1+q2) , 0],
                  [0.2*np.cos(q1) + 0.2*np.cos(q1+q2), 0.2*np.cos(q1+q2), 0],
                  [0, 0, -1],
                  [0, 0, 0],
                  [0, 0, 0],
                  [1, 1, 0]])

    print J.round(2)

    return str(J.round(2))

def forward_kin_server():

    rospy.init_node('forward_kin_server', anonymous=True)
    forward_kin = rospy.Service('forward_kin', ForwardKin, handle_forward_kin)
    jacobian = rospy.Service('jacobian', Jacobian, handle_jacobian)
    print "Ready to calculate forward kinematics"

    rospy.spin()


if __name__ == '__main__':
    forward_kin_server()



