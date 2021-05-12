#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
from rbe500_project.msg import joint_velocities, joint_angles
from rbe500_project.srv import EEToJointVels,EEToJointVelsResponse, JointToEEVels, JointToEEVelsResponse 
#from rbe500_project.msg import joint_angles

def ee_to_joint_vels(req):
    response = joint_velocities()

    '''
    theta1 = req.joint_set_points.theta1
    theta2 = req.joint_set_points.theta2
    d3 = req.joint_set_points.d3

    # Maybe change to joint limits to be referenced from xacro file
    if ((theta1 <= math.pi and theta1 >= -math.pi) 
    and (theta2 <= math.pi and theta2 >= -math.pi) 
    and (d3 <= 1 and d3 >= -1)):

        response = True

        set_points.theta1 = theta1
        set_points.theta2 = theta2
        set_points.d3 = d3

        pub_setPoint.publish(set_points)
    '''

    return EEToJointVelsResponse(response)

def joint_to_ee_vels(req):

    pass

def velocity_kinematics_converter_server():
    rospy.init_node('velocity_kinematics_converter_server')

    ee_to_joint_vel_s = rospy.Service('ee_vels_to_joint_vels', EEToJointVels, ee_to_joint_vels)
    joint_to_ee_vel_s = rospy.Service('joint_vels_to_ee_vels', EEToJointVels, joint_to_ee_vels)


    print("Ready to convert velocities.")
    rospy.spin()

if __name__ == "__main__":
    velocity_kinematics_converter_server()