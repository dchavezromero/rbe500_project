#!/usr/bin/env python
import rospy
import math
import numpy as np
from gazebo_msgs.srv import *
from std_msgs.msg import Float64


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

def callback(msg):
    d3 = get_position('d3')
    rospy.loginfo(msg.data)


def main():
    rospy.init_node("position_controller")
    sub = rospy.Subscriber("/scara/set_point", Float64, callback)
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass