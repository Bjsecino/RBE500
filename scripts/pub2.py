#!/usr/bin/env python

from rbe_500.srv import pose,poseResponse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import time

pub = rospy.Publisher('/custom_scara/joint3_position_controller/command', Float64, queue_size=10)

def publishJointState(req):
	global pub
	#rate = rospy.Rate(100)
	pub.publish(req.q3)
	#time.sleep(1)
	return posResponse(req.q3)
	
def controller_server():
    rospy.init_node('controller_server')
    s = rospy.Service('controller', pose, publishJointState)
    print "Ready"
    rospy.spin()

if __name__ == "__main__":
	controller_server()

	
    
