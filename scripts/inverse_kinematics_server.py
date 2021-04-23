#!/usr/bin/env python

from __future__ import print_function

from rbe500_project.srv import InvKin,InvKinResponse
from rbe500_project.msg import joint_angles
import rospy
import math
from gazebo_msgs.srv import *

#pub_theta1 = rospy.Publisher("/scara/theta1_position_controller/command", Float64, queue_size=1)
#pub_theta2 = rospy.Publisher("/scara/theta2_position_controller/command", Float64, queue_size=1)
#pub_d3 = rospy.Publisher("/scara/d3_position_controller/command", Float64, queue_size=1)

result = joint_angles()

def write_position():
    rospy.wait_for_service('/gazebo/set_model_configuration')

    try:
        joint_call = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        joint_req = SetModelConfigurationRequest()
        joint_req.model_name = 'scara'
        joint_req.urdf_param_name = ''
        joint_req.joint_names = ['theta1', 'theta2', 'd3'] #list
        joint_req.joint_positions = [result.theta1, result.theta2, result.d3] #list
        res = joint_call(joint_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def calc_inv_kin(req):

    x = req.x
    y = req.y
    z = req.z

    L = 1

    D2 = -(2*L**2 -(x**2 + y**2))/(2*(L**2))
    C2 = math.sqrt(1-(D2**2))

    theta2 = math.atan2(C2, D2)
    
    alpha = math.atan2(y,x)
    D1 = (x**2+y**2)/(2*L*math.sqrt(x**2+y**2))
    C1 = math.sqrt(1-(D1**2))
    beta = math.atan2(C1, D1)
    
    theta1 = alpha - beta

    d3 = L - z

    result.theta1 = theta1
    result.theta2 = theta2
    result.d3 = d3

    # Move joints to calculated values
    write_position()
    #pub_theta1.publish(result.theta1)
    #pub_theta2.publish(result.theta2)
    #pub_d3.publish(result.d3)

    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    calc_inv_kin_server()