#!/usr/bin/env python
import rospy
from rbe_500.srv import position_controller_reference,position_controller_referenceResponse

from std_msgs.msg import Float64

def handle_set_ref(data):
	rospy.loginfo(data)
	pub1.publish(data.ref1)
	pub2.publish(data.ref2)
	pub3.publish(data.ref3)
	ret.resp = 1
	return ret


def position_control_server():


	s = rospy.Service('set_ref_pos',position_controller_reference,handle_set_ref)
	print("ready to set reference")
	rospy.spin()


if __name__=="__main__" : 
	rospy.init_node('reference_pos_server',anonymous = True)


	pub1 = rospy.Publisher('/custom_scara/joint1_position_controller/command',Float64,queue_size=1)
	pub2 = rospy.Publisher('/custom_scara/joint2_position_controller/command',Float64,queue_size=1)
	pub3 = rospy.Publisher('/custom_scara/joint3_position_controller/command',Float64,queue_size=1)
	ret = position_controller_referenceResponse()
	position_control_server()

