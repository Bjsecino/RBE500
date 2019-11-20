#!/usr/bin/env python

import rospy
import math
import numpy as np
from rbe_500.srv import InverseKinematics, InverseKinematicsResponse

def rot_x(a):
	R = np.asmatrix([[1, 0, 0],[0, math.cos(a), -1*math.sin(a)],[0, math.sin(a), math.cos(a)]])
	return R

def rot_y(a):
	R = np.asmatrix([[math.cos(a), 0, math.sin(a)],[0, 1, 0], [-1*math.sin(a), 0, math.cos(a)]])
	return R

def rot_z(a):
	R = np.asmatrix([[math.cos(a), -1*math.sin(a), 0 ],[math.sin(a),math.cos(a), 0 ],[0,0,1]])
	return R

def check(R,R_new) :
	match = 1
	for i in range(R.shape[0]) :
		for j in range(R.shape[1]) :
			if (R[i,j] != R_new[i,j]) :
				# print("i : %s, j : %s"%(i,j))
				# print("values : %s %s "%(R[i,j],R_new[i,j]))
				match = 0
				break
	return match

def calc_joints(D2,D,data) :
	joints = InverseKinematicsResponse()

	joints.q2 = math.atan2(D2,D) 
	joints.q1 = math.atan2(data.y,data.x) - math.atan2(0.2*math.sin(joints.q2),0.2+0.2*math.cos(joints.q2))
	joints.q3 = 0.1 - data.z

	Rx = rot_x(math.pi)
	Ry = rot_y(0)
	Rz = rot_z(-1*(joints.q1+joints.q2))
	R_from_joints = np.matmul(np.matmul(Rx,Ry), Rz).round(2)
	# print(R_from_joints)

	return joints,R_from_joints


def handle_inversekinematics(data) :
	# print("Got [%s %s %s %s %s %s] as the end effector pose"%(data.x,data.y,data.z,data.t1,data.t2,data.t3))
	data.t1 = np.radians(data.t1)
	data.t2 = np.radians(data.t2)
	data.t3 = np.radians(data.t3)

	joints = InverseKinematicsResponse()
	
	r = data.x*data.x + data.y*data.y
	D = (r - 0.08)/0.08
	Rx = rot_x(data.t1)
	Ry = rot_y(data.t2)
	Rz = rot_z(data.t3)
	R = np.matmul(np.matmul(Rx,Ry),Rz).round(2)
	# print(R)
	try : 
		D2_1 = math.sqrt(1-D*D)
		D2_2 = -1 * D2_1
	except ValueError:
		print
	# print("D2   %s"%D2_1)
	# print("D2_2 %s"%D2_2)
	# print("D %s"%D)

	joints, R_from_joints = calc_joints(D2_1,D,data)

	if check(R,R_from_joints) == 0 : 
		# print("Wrong")
		joints, R_from_joints = calc_joints(D2_2,D,data)
			

	print("joints : %s %s %s"%(round(joints.q1*180/math.pi,2), round(joints.q2*180/math.pi,2), joints.q3))
	return joints



def inverse_kinematics() :
	rospy.init_node('inversekinematics_server')
	s = rospy.Service('invkin_joint_positions',InverseKinematics,handle_inversekinematics)
	print ("Ready to calculate")
	rospy.spin()



if __name__=="__main__" :
	inverse_kinematics()