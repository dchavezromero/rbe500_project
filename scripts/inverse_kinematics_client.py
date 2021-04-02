#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from rbe500_project.srv import *

def calc_inv_kin_client(x, y, z):
    rospy.wait_for_service('inverse_kinematics')
    try:
        inverse_kinematics = rospy.ServiceProxy('inverse_kinematics', InvKin)
        resp1 = inverse_kinematics(x, y, z)
        return resp1.joint_vals
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)

    print("Requesting %s,%s,%s"%(x, y, z))
    print(calc_inv_kin_client(x, y, z))