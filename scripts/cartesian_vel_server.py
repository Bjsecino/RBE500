#!/usr/bin/env python

import rospy
import csv
import time
from rbe_500.srv import CartesianVel, CartesianVelResponse
from sensor_msgs.msg import JointState
from rbe_500.srv import JointVel, JointVelResponse
from rbe_500.srv import velocity_controller_reference,velocity_controller_referenceResponse
import numpy as np
begin_time = time.time()

global dx_des
global dy_des
global dz_des
# global ref_vel_list

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
	global begin_time
	global ref_vel_list
	# ref_vel_list = []
	ser = rospy.ServiceProxy('/ee_vel_to_joint_vel', JointVel)
	ref_vel = rospy.ServiceProxy('/set_ref_vel', velocity_controller_reference)

	global dx_des
	global dy_des
	global dz_des

	q1 = msg.position[0]
	q2 = msg.position[1]
	q3 = msg.position[2]

	q1_dot = msg.velocity[0]
	q2_dot = msg.velocity[1]
	q3_dot = msg.velocity[2]

	joint_vels = ser(q1, q2, q3, dx_des, dy_des, dz_des)

	dq1_des = joint_vels.joint_vel[0]
	dq2_des = joint_vels.joint_vel[1]
	dq3_des = joint_vels.joint_vel[2]
	
	end_time = time.time()
	time_used = end_time - begin_time
	if dq1_des!= 0 or dq2_des !=0 or dq3_des != 0 : 
		if q1!=1.57079 or q2!=0 : 
			write_csv([dq1_des,dq2_des,dq3_des,q1_dot,q2_dot,q3_dot])

	ref_vel(dq1_des, dq2_des, dq3_des)


def cartesian_vel():
	rospy.init_node('cartesian_vel_server')
	rospy.wait_for_service('/ee_vel_to_joint_vel')
	rospy.wait_for_service('/set_ref_vel')
	rospy.Subscriber("/custom_scara/joint_states", JointState, callback_jointstates)
	s = rospy.Service('cartesian_vel',CartesianVel,handle_cartesian_vel)
	
	print "ready to drive"
	rospy.spin()
	
def write_csv(arr):
	# np.savetxt('ref_vel.txt',arr,delimiter)
	path  = '/home/madhan/catkin_ws/src/RBE500/scripts/ref_vel.csv'
	print("writing")
	with open(path,mode='a') as f:
		csv_write = csv.writer(f)
 #    	# data_row = arr
		csv_write.writerow(arr)


if __name__=="__main__" :
	cartesian_vel()
	# global ref_vel_list
	# ref_vel_list = []
