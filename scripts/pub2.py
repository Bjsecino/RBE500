#!/usr/bin/env python

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import time

def callback_jointstates(msg):
	global pub
	Q3 = msg.position[0]
	print("Q3 from gazebo: " + str("%2.3f" %
                           Q3))
	

if __name__ == "__main__":
	rospy.init_node('testNode')
	print("Printing values from Gazebo")
	rospy.Subscriber("/custom_scara/joint_states",
                     JointState, callback_jointstates)
	rospy.spin()
