#!/usr/bin/env python

from __future__ import print_function

import rospy
from rbe500_project.srv import UpdateVelocities,UpdateVelocitiesResponse
from rbe500_project.msg import ee_velocities

pub_refVels = rospy.Publisher("/scara/ref_velocities", ee_velocities, queue_size=1)
ref_vels = ee_velocities()

def callback(req):

    ref_vels.x_velocity = req.ee_vels.x_velocity
    ref_vels.y_velocity = req.ee_vels.y_velocity
    ref_vels.z_velocity = req.ee_vels.z_velocity
    ref_vels.roll_velocity = req.ee_vels.roll_velocity
    ref_vels.twist_velocity = req.ee_vels.twist_velocity
    ref_vels.yaw_velocity = req.ee_vels.yaw_velocity

    response = True

    pub_refVels.publish(ref_vels)

    return UpdateVelocitiesResponse(response)

def velocity_controller_server():
    rospy.init_node('velocity_controller_server')
    s = rospy.Service('velocity_controller', UpdateVelocities, callback)
    print("Ready to update ref velocity.")
    rospy.spin()

if __name__ == "__main__":
    velocity_controller_server()