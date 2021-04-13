#!/usr/bin/env python
import rospy
import math
import numpy as np
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose

result = Pose()

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    quaternions = qx, qy, qz, qw
    return quaternions

def get_quaternions(rot_matrix):
    roll = math.atan2(rot_matrix[7], rot_matrix[8])
    pitch = math.atan2(-rot_matrix[6], math.sqrt(pow(rot_matrix[7], 2) + pow(rot_matrix[8], 2)))
    yaw = math.atan2(rot_matrix[3], rot_matrix[0])

    quaternions = euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
    return quaternions

def multiply(X, Y):
    matrix = [[sum(a*b for a, b in zip(X_row, Y_col)) for Y_col in zip(*Y)] for X_row in X]
    return matrix

def calculate_forward_kin(joints):
    T01 = caclDH(0, joints[0])
    T02 = multiply(T01, caclDH(1, joints[1]))
    T03 = multiply(T02, caclDH(2, joints[2]))

    return T03

def caclDH(joint, angle):
    
    L = 1

    dh_const = [[angle, L, L, 0],
                [angle, 0, L, math.pi],
                [0, angle, 0, 0]]

    transform = [[math.cos(dh_const[joint][0]),
                  -math.sin(dh_const[joint][0])*math.cos(dh_const[joint][3]),
                  math.sin(dh_const[joint][0])*math.sin(dh_const[joint][3]),
                  dh_const[joint][2]*math.cos(dh_const[joint][0])],
                 [math.sin(dh_const[joint][0]),
                  math.cos(dh_const[joint][0])*math.cos(dh_const[joint][3]),
                  -math.cos(dh_const[joint][0])*math.sin(dh_const[joint][3]),
                  dh_const[joint][2]*math.sin(dh_const[joint][0])],
                 [0,
                  math.sin(dh_const[joint][3]),
                  math.cos(dh_const[joint][3]),
                  dh_const[joint][1]],
                 [0, 0, 0, 1]]
    return transform


def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def callback():
    d3 = get_position('d3')
    theta1 = get_position('theta1')
    theta2 = get_position('theta2')

    joint_angles = theta1, theta2, d3

    trans_matrix = calculate_forward_kin(joint_angles)
    ee_pos = trans_matrix[0][3], trans_matrix[1][3], trans_matrix[2][3]
    rot_matrix = trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2], \
                 trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2], \
                 trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]

    quat = get_quaternions(rot_matrix)

    result.position.x = ee_pos[0]
    result.position.y = ee_pos[1]
    result.position.z = ee_pos[2]
    result.orientation.x = quat[0]
    result.orientation.y = quat[1]
    result.orientation.z = quat[2]
    result.orientation.w = quat[3]


def main():
    pub = rospy.Publisher("/scara/ee_pose", Pose, queue_size=1)
    rospy.init_node("forward_kinematics")
    r = rospy.Rate(15)

    while not rospy.is_shutdown():

        callback()
        rospy.loginfo(result)
        pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass