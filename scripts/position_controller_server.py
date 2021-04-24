#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import numpy as np
from rbe500_project.srv import MoveJoint,MoveJointResponse
from std_msgs.msg import Float64

pub_setPoint = rospy.Publisher("/scara/set_point", Float64, queue_size=1)

def callback(req):
    response = False

    if (req.set_point <= 1 and req.set_point >= 0):
        response = True
        pub_setPoint.publish(req.set_point)

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    rospy.spin()

if __name__ == "__main__":
    position_controller_server()