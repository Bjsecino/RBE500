#!/usr/bin/env python

import numpy as np
import rospy
import math
import time

from sensor_msgs.msg import JointState
# These services are defined in SRV folder
from rbe_500.srv import ForwardKin,ForwardKinResponse,InverseKinematics,InverseKinematicsResponse  

# (Q3.a)
def callback_jointstates(msg):  

    Q1 = msg.position[0]
    Q2 = msg.position[1]
    Q3 = msg.position[2]

    try:
        # To check from FK node (Q3.b)
        FK_Service = rospy.ServiceProxy(
            'forward_kin_server', ForwardKin)  # Get the service object
        
        pose_received = FK_Service(q1,  # Joint 1  ## Call the service
                                         q2,  # Joint 2
                                         q3)  # Joint 3
        
        IK_Service = rospy.ServiceProxy(
            'invkin_joint_positions', InverseKinematics)  # Get the service object

        joint_params_received = IK_Service(pose_received.x,  # Call the service
                                                        pose_received.y,
                                                        pose_received.z,
                                                        pose_received.t1,
                                                        pose_received.t2,
                                                        pose_received.t3)
                                                        
        
        print("Q1 Gazebo: " + str("%5.3f" %
                           Q1) + ', Node:' + str(joint_params_received.q1))
        print("Q2 Gazebo: " + str("%5.3f" %
                           Q2) + ', Node:' + str(joint_params_received.q2))
        print("Q3 Gazebo: " + str("%5.3f" %
                           Q3) + ', Node: ' + str(joint_params_received.q3))

        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    rospy.init_node('processNode')
    # Wait for FK and IK Node to start
    rospy.wait_for_service('forward_kin_server')
    rospy.wait_for_service('invkin_joint_positions')

    print("Starting to Test Forward and Inverse Kinematics Nodes")

    rospy.Subscriber("/scara/joint_states", JointState, callback_jointstates)

    rospy.spin()
