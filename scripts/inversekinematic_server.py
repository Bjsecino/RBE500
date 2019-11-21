#!/usr/bin/env python

import rospy
import math
import numpy as np
from rbe_500.srv import InverseKinematics, InverseKinematicsResponse

#link lengths
l1_v = 1
l1_h = 1
l2 = 1
l3 = 1
base_length = 1



def rot_x(a):
	R = np.asmatrix([[1, 0, 0],[0, math.cos(a), -1*math.sin(a)],[0, math.sin(a), math.cos(a)]])
	return R

def rot_y(a):
	R = np.asmatrix([[math.cos(a), 0, math.sin(a)],[0, 1, 0], [-1*math.sin(a), 0, math.cos(a)]])
	return R

def rot_z(a):
	R = np.asmatrix([[math.cos(a), -1*math.sin(a), 0 ],[math.sin(a),math.cos(a), 0 ],[0,0,1]])
	return R

def check(joints1,joints2,data) :
	
	sum_angle1 = np.mod((np.degrees(joints1.q1 + joints1.q2)),360)
	sum_angle2 = np.mod((np.degrees(joints2.q1 + joints2.q2)),360)
	angle1 = (np.mod((np.degrees(data.t3)),360))
	# print("joints1 : %s"%joints1)
	# print("joints2 : %s"%joints2)
	# print"sum1 : %s"%sum_angle1
	# print"sum2 : %s"%sum_angle2
	# print("data.t3 : %s"%data.t3)
	# print("angle1 : %s"%angle1)
	
	if (abs( angle1 - sum_angle1 ) > abs( angle1 - sum_angle2)) :
		return joints2
	else : 
		return joints1 


	# for i in range(R.shape[0]) :
	# 	for j in range(R.shape[1]) :
	# 		if (abs(R[i,j] - R_new[i,j]) > 0.05) :
	# 			# print("i : %s, j : %s"%(i,j))
	# 			# print("values : %s %s "%(R[i,j],R_new[i,j]))
	# 			match = 0
	# 			break
	# return joints1

def calc_joints(D2,D,data) :
	joints = InverseKinematicsResponse()
	
	joints.q2 = math.atan2(D2,D) 
	joints.q1 = math.atan2(data.y,data.x) - math.atan2(l2*math.sin(joints.q2),l1_h+l2*math.cos(joints.q2))
	joints.q3 = data.z - ( l1_v + base_length - l3 )

	Rx = rot_x(0)
	Ry = rot_y(0)
	Rz = rot_z(joints.q1+joints.q2)
	R_from_joints = np.matmul(np.matmul(Rz,Ry), Rx)
	# print(R_from_joints.round(2))
	return joints,R_from_joints

def handle_inversekinematics(data) :
	print("Got [%s %s %s %s %s %s] as the end effector pose"%(data.x,data.y,data.z,data.t1,data.t2,data.t3))
	# data.t1 = np.radians(data.t1)
	# data.t2 = np.radians(data.t2)
	# data.t3 = np.radians(data.t3)

	joints = InverseKinematicsResponse()
	
	r = data.x*data.x + data.y*data.y
	D = (r - l1_h*l1_h - l2*l2)/(2*l1_h*l2)
	Rx = rot_x(data.t1)
	Ry = rot_y(data.t2)
	Rz = rot_z(data.t3)
	R = np.matmul(np.matmul(Rz,Ry),Rx)
	# print("----------")
	# print(R.round(2))
	# print("++++")
	try : 
		D2_1 = math.sqrt(1-D*D)
		D2_2 = -1 * D2_1
	except ValueError:
		print
	joints1, R_from_joints1 = calc_joints(D2_1,D,data)
	joints2, R_from_joints2 = calc_joints(D2_2,D,data)
	joints = check(joints1,joints2,data)
	print joints
	# if check(R,R_from_joints) == 0 : 
	return joints
		# print("Wrong")
	# print("D2   %s"%D2_1)
	# print("D2_2 %s"%D2_2)
	# print("D %s"%D)


					

	# print("joints : %s %s %s"%(round(joints.q1*180/math.pi,2), round(joints.q2*180/math.pi,2), joints.q3))



def inverse_kinematics() :
	rospy.init_node('inversekinematic_server')
	s = rospy.Service('invkin_joint_positions',InverseKinematics,handle_inversekinematics)
	# print l2
	print ("Ready to calculate")
	rospy.spin()



if __name__=="__main__" :
	inverse_kinematics()