#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import numpy as np
from gazebo_msgs.srv import *
from rbe500_project.srv import MoveJoint,MoveJointResponse

def write_effort(joint, effort, duration):
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    try:
        joint_call = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        joint_req = ApplyJointEffortRequest()
        joint_req.joint_name = joint
        joint_req.effort = effort
        joint_req.duration = duration
        res = joint_call(joint_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def callback(req):
    d3 = get_position('d3')
    set_point = req.set_point

    response = True

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    rospy.spin()

if __name__ == "__main__":
    position_controller_server()