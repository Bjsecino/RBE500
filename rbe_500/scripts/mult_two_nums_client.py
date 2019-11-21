#!/usr/bin/env python

import sys
import rospy
from rbe_500.srv import *

def mult_two_nums_client(x, y):
    rospy.wait_for_service('mult_two_nums')
    try:
        mult_two_nums = rospy.ServiceProxy('mult_two_nums', MultTwoNums)
        resp1 = mult_two_nums(x, y)
        return resp1.product #tutorial used resp1.sum What does this do? Is it from the srv file?
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e #service call failed 'MultTwoNumsRequest' object has no attribute 'a'

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s * %s"%(x, y)
    print "%s * %s = %s"%(x, y, mult_two_nums_client(x, y))
