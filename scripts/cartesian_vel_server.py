#!/usr/bin/env python

import rospy
from rbe_500.srv import CartesianVel, CartesianVelResponse
from sensor_msgs.msg import JointState
from rbe_500.srv import JointVel, JointVelResponse
from rbe_500.srv import velocity_controller_reference,velocity_controller_referenceResponse


global dx_des
global dy_des
global dz_des

dx_des = 0
dy_des = 0
dz_des = 0

def handle_cartesian_vel(data):
	global dx_des
	global dy_des
	global dz_des

	dx_des = data.dxdes
	dy_des = data.dydes
	dz_des = data.dzdes

	return CartesianVelResponse()

	
def callback_jointstates(msg):

	ser = rospy.ServiceProxy('/ee_vel_to_joint_vel', JointVel)
	ref_vel = rospy.ServiceProxy('/set_ref_vel', velocity_controller_reference)

	global dx_des
	global dy_des
	global dz_des

	q1 = msg.position[0]
	q2 = msg.position[1]
	q3 = msg.position[2]

	joint_vels = ser(q1, q2, q3, dx_des, dy_des, dz_des)

	dq1_des = joint_vels.joint_vel[0]
	dq2_des = joint_vels.joint_vel[1]
	dq3_des = joint_vels.joint_vel[2]
	
	#print("%s %s %s" % (dq1_des, dq2_des, dq3_des))
	#print q2

	ref_vel(dq1_des, dq2_des, dq3_des)


def cartesian_vel():
	rospy.init_node('cartesian_vel_server')
	rospy.wait_for_service('/ee_vel_to_joint_vel')
	rospy.wait_for_service('/set_ref_vel')
	rospy.Subscriber("/custom_scara/joint_states", JointState, callback_jointstates)
	s = rospy.Service('cartesian_vel',CartesianVel,handle_cartesian_vel)
	
	print "ready to drive"
	rospy.spin()


if __name__=="__main__" :
	cartesian_vel()
