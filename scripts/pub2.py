#!/usr/bin/env python

from rbe_500.srv import pose,poseResponse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import time

pub = rospy.Publisher('/custom_scara/joint3_position_controller/command', Float64, queue_size=10)

def callback_jointstates(msg):
	global pub
	Q3 = msg.position[2]
	#rate = rospy.Rate(10)
	#pub.publish(req.q3)
	time.sleep(0.2)
	print("Q3 from gazebo: " + str("%2.3f" %
                           Q3))
	


if __name__ == "__main__":
	rospy.init_node('testNode')
	print("Printing values from Gazebo")
	rospy.Subscriber("/custom_scara/joint_states",
                     JointState, callback_jointstates)
	rospy.spin()
