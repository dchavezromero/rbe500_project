#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
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
    T12 = caclDH(0, joints[0])
    T13 = multiply(T12, caclDH(1, joints[1]))
    T14 = multiply(T13, caclDH(2, joints[2]))
    T15 = multiply(T14, caclDH(3, joints[3]))
    T16 = multiply(T15, caclDH(4, joints[4]))
    T17 = multiply(T16, caclDH(5, joints[5]))

    return T17

def caclDH(joint, angle):

    dh_const = [[angle, 475, 150, math.pi/2],
                [angle + math.pi/2, 0, 600, 0],
                [angle, 0, 0, math.pi/2],
                [angle, 720, 120, -math.pi/2],
                [angle, 0, 0, math.pi/2],
                [angle + math.pi, 85, 0, 0]]

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


def callback(msg):
    d3 = msg.position[0]


    rospy.loginfo(d3)

def main():
    rospy.init_node("forward_kinematics")
    #pub = rospy.Publisher("fowardkin", Pose, queue_size=1)
    sub = rospy.Subscriber("/scara/joint_states", JointState, callback)
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        #pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass