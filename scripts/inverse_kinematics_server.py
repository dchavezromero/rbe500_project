#!/usr/bin/env python

from __future__ import print_function

from rbe500_project.srv import InvKin,InvKinResponse
from rbe500_project.msg import joint_angles
import rospy

result = joint_angles()

def calc_inv_kin(req):
    result.theta1 = req.x
    result.theta2 = req.y
    result.d3 = req.z
    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    calc_inv_kin_server()