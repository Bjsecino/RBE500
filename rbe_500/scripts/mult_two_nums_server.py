#!/usr/bin/env python

from rbe_500.srv import MultTwoNums, MultTwoNumsResponse
import rospy

def handle_mult_two_nums(req):
	print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a * req.b))
	return MultTwoNumsResponse(req.a * req.b)

def mult_two_nums_server():
	rospy.init_node('mult_two_nums_server')
	s = rospy.Service('mult_two_nums', MultTwoNums, handle_mult_two_nums)
	print "Ready to multiply two numbers."
	rospy.spin()

if __name__ == "__main__":
	mult_two_nums_server()
