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
            'forward_kin', ForwardKin)  # Get the service object
        
        pose_received = FK_Service(Q1,  # Joint 1  ## Call the service
                                         Q2,  # Joint 2
                                         Q3)  # Joint 3
        
        # print("euler : %s %s"%(pose_received.pose[3],pose_received.pose[4]))

        IK_Service = rospy.ServiceProxy(
            'invkin_joint_positions', InverseKinematics)  # Get the service object

        joint_params_received = IK_Service(pose_received.pose[0],  # Call the service
                                                        pose_received.pose[1],
                                                        pose_received.pose[2],
                                                        pose_received.pose[3],
                                                        pose_received.pose[4],
                                                        pose_received.pose[5])
                                                        
        # if (round(Q1,3) != round(joint_params_received.q1,3) or round(Q2,3) != round(joint_params_received.q2,3) or round(Q3,3) != round(joint_params_received.q3,3)) :
        #     print("Not matching")
        print("Q1 Gazebo: " + "%.3f" %
                           (Q1*180/math.pi) + ', Node: %f' % (round((joint_params_received.q1*180/math.pi),3)))
        print("Q2 Gazebo: " + "%.3f" %
                           (Q2*180/math.pi) + ', Node: %f' % (round((joint_params_received.q2*180/math.pi),3)))
        print("Q3 Gazebo: " + "%.3f" %
                           (Q3) + ', Node: %f' % (round(joint_params_received.q3,3)))
        print("-------------------------")

        # print("Q1 : %s, Q2 : %s, Q3 : %s"%(Q1,Q2,Q3))
        # print("q1 : %s, q2 : %s, q3 : %s"%(joint_params_received.q1,joint_params_received.q2,joint_params_received.q3))
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    rospy.init_node('process')
    # Wait for FK and IK Node to start
    rospy.wait_for_service('forward_kin')
    rospy.wait_for_service('invkin_joint_positions')

    print("Starting to Test Forward and Inverse Kinematics Nodes")

    rospy.Subscriber("/custom_scara/joint_states", JointState, callback_jointstates)

    rospy.spin()