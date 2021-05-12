#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import sympy as sym
from gazebo_msgs.srv import *
from rbe500_project.msg import joint_velocities, ee_velocities
from rbe500_project.srv import EEToJointVels,EEToJointVelsResponse, JointToEEVels, JointToEEVelsResponse 

def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def caclDHSymbolic(joint, angle):
    
    L = 1

    dh_const = [[angle, L, L, 0],
                [angle, 0, L, math.pi],
                [0, angle, 0, 0]]

    transform = sym.Matrix([[sym.cos(dh_const[joint][0]),
                  -sym.sin(dh_const[joint][0])*sym.cos(dh_const[joint][3]),
                  sym.sin(dh_const[joint][0])*sym.sin(dh_const[joint][3]),
                  dh_const[joint][2]*sym.cos(dh_const[joint][0])],
                 [sym.sin(dh_const[joint][0]),
                  sym.cos(dh_const[joint][0])*sym.cos(dh_const[joint][3]),
                  -sym.cos(dh_const[joint][0])*sym.sin(dh_const[joint][3]),
                  dh_const[joint][2]*sym.sin(dh_const[joint][0])],
                 [0,
                  sym.sin(dh_const[joint][3]),
                  sym.cos(dh_const[joint][3]),
                  dh_const[joint][1]],
                 [0, 0, 0, 1]])
    return transform

def get_jacob():

    theta1_pos = get_position('theta1')
    theta2_pos = get_position('theta2')
    d3_pos = get_position('d3')

    joint_pos = theta1_pos, theta2_pos, d3_pos


    # Declare symbolic variables
    theta1, theta2, d3 = sym.symbols("theta1 theta2 d3")

    #Forwd kin
    T01 = caclDHSymbolic(0, theta1)
    T02 = T01 * caclDHSymbolic(1, theta2)
    T03 = T02 * caclDHSymbolic(2, d3)

    # Delete last row of all matrices
    T01.row_del(-1)
    T02.row_del(-1)
    T03.row_del(-1)

    Pe = T03.col(-1)

    # Diffirentiate wtr each symbolic var
    upper_J_col_1 = sym.diff(Pe, theta1)
    upper_J_col_2 = sym.diff(Pe, theta2)
    upper_J_col_3 = sym.diff(Pe, d3)

    # Getting cols from rot matrices
    z1 = T01.col(2)
    z2 = T02.col(2)
    z3 = T03.col(2)

    # Declaration of jacobian as a matrix
    J = sym.Matrix([[upper_J_col_1, upper_J_col_2, upper_J_col_3],
                    [z1, z2, z3]])

    val_J = J.subs({theta1: joint_pos[0], theta2: joint_pos[1], d3: joint_pos[2]})

    return val_J


def ee_to_joint_vels(req):
    response = joint_velocities()
    x_vel = req.ee_vels.x_velocity
    y_vel = req.ee_vels.y_velocity
    z_vel = req.ee_vels.z_velocity
    roll_vel = req.ee_vels.roll_velocity
    twist_vel = req.ee_vels.twist_velocity
    yaw_vel = req.ee_vels.yaw_velocity

    ee_vels = sym.Matrix([[x_vel], [y_vel], [z_vel], [roll_vel], [twist_vel], [yaw_vel]])

    J = get_jacob()

    J_pseudo_inv = (J.T * J)**-1 * J.T

    joint_vels = J_pseudo_inv*ee_vels

    response.theta1_velocity = joint_vels[0]
    response.theta2_velocity = joint_vels[1]
    response.d3_velocity = joint_vels[2]

    return EEToJointVelsResponse(response)

def joint_to_ee_vels(req):
    response = ee_velocities()
    theta1_vel = req.joint_vels.theta1_velocity
    theta2_vel = req.joint_vels.theta2_velocity
    d3_vel = req.joint_vels.d3_velocity

    joint_vels = sym.Matrix([[theta1_vel], [theta2_vel], [d3_vel]])

    J = get_jacob()

    ee_vels = J*joint_vels

    response.x_velocity = ee_vels[0]
    response.y_velocity = ee_vels[1]
    response.z_velocity = ee_vels[2]
    response.roll_velocity = ee_vels[3]
    response.twist_velocity = ee_vels[4]
    response.yaw_velocity = ee_vels[5]

    return JointToEEVelsResponse(response)

def velocity_kinematics_converter_server():
    rospy.init_node('velocity_kinematics_converter_server')

    ee_to_joint_vel_s = rospy.Service('ee_vels_to_joint_vels', EEToJointVels, ee_to_joint_vels)
    joint_to_ee_vel_s = rospy.Service('joint_vels_to_ee_vels', JointToEEVels, joint_to_ee_vels)


    print("Ready to convert velocities.")
    rospy.spin()

if __name__ == "__main__":
    velocity_kinematics_converter_server()