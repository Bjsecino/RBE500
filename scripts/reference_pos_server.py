#!/usr/bin/env python
import rospy
from rbe_500.srv import position_controller_reference,position_controller_referenceResponse
from std_msgs.msg import Float64

def handle_set_ref(data):
	rospy.loginfo(data.ref)
	pub.publish(data.ref)
	ret.resp = 1
	return ret


def position_control_server():
	s = rospy.Service('set_ref_pos',position_controller_reference,handle_set_ref)
	print("ready to set reference")
	rospy.spin()

if __name__=="__main__" : 
	rospy.init_node('reference_pos_server',anonymous = True)
	pub = rospy.Publisher('/custom_scara/joint3_position_controller/command',Float64,queue_size=1)
	ret = position_controller_referenceResponse()
	position_control_server()

