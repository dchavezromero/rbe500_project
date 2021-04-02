#!/usr/bin/env python

from __future__ import print_function

from rbe500_project.srv import InvKin,InvKinResponse
from rbe500_project.msg import joint_angles
import rospy
from std_msgs.msg import Float64

pub_theta1 = rospy.Publisher("/scara/theta1_position_controller/command", Float64, queue_size=1)
pub_theta2 = rospy.Publisher("/scara/theta2_position_controller/command", Float64, queue_size=1)
pub_d3 = rospy.Publisher("/scara/d3_position_controller/command", Float64, queue_size=1)
result = joint_angles()

def calc_inv_kin(req):
    result.theta1 = req.x
    result.theta2 = req.y
    result.d3 = req.z

    pub_theta1.publish(result.theta1)
    pub_theta2.publish(result.theta2)
    pub_d3.publish(result.d3)

    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    calc_inv_kin_server()