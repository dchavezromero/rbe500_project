#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
from rbe500_project.srv import MoveJoint,MoveJointResponse
from rbe500_project.msg import joint_angles

pub_setPoint = rospy.Publisher("/scara/set_points", joint_angles, queue_size=1)
set_points = joint_angles()

def callback(req):
    response = False

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

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    rospy.spin()

if __name__ == "__main__":
    position_controller_server()