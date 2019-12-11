#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import SwitchController
from rbe_500.srv import ContSwitcher, ContSwitcherResponse

def handle_p_to_v(data):
	rospy.wait_for_service('/custom_scara/controller_manager/switch_controller')

	switch_controller = rospy.ServiceProxy('/custom_scara/controller_manager/switch_controller', SwitchController)


	ret = switch_controller(['joint1_velocity_controller', 'joint2_velocity_controller', 'joint3_velocity_controller'], 
				['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller'], 1)
	return ContSwitcherResponse()

def handle_v_to_p(data):
	rospy.wait_for_service('/custom_scara/controller_manager/switch_controller')

	switch_controller = rospy.ServiceProxy('/custom_scara/controller_manager/switch_controller', SwitchController)


	ret = switch_controller(['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller'],
				['joint1_velocity_controller', 'joint2_velocity_controller', 'joint3_velocity_controller'], 1)
	return ContSwitcherResponse()
	

def switch_controller():
	rospy.init_node('switch_controller_server', anonymous = True)
	p_to_v = rospy.Service('joint_pos_to_vel', ContSwitcher, handle_p_to_v)
	v_to_p = rospy.Service('joint_vel_to_pos', ContSwitcher, handle_v_to_p)

	print ("ready to switch controllers")

	rospy.spin()


if __name__=="__main__" : 
	switch_controller()

	
