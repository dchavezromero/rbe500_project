#!/usr/bin/env python
import rospy
import math
import numpy as np
import csv
import os
import time
from gazebo_msgs.srv import *
from std_msgs.msg import Float64

Kp = 180
Kd = 15
ros_rate = 10.0
sampling_rate_ms = (1/ros_rate) * 1000.0
last_position = 0.0
last_set_point = 0.0
record_time_interval = 15

start_time = 0
timer_started = False
times = []
set_points = []
curr_points = []

def reset_timer():
    global times, set_points, curr_points, timer_started

    times = []
    set_points = []
    curr_points = []
    timer_started = False

def record_data(time, set_point, curr_point):

    with open('position_controller_data.csv', 'wb') as csv_file:

        writer = csv.writer(csv_file, delimiter=',')

        for i in range(len(time)):
            writer.writerow((time[i], set_point[i], curr_point[i]))


def write_effort(effort, duration_sec):
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    try:
        joint_call = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        joint_req = ApplyJointEffortRequest()
        joint_req.joint_name = 'd3'
        joint_req.effort = effort
        joint_req.duration.nsecs = duration_sec * 100000000
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

def do_pd_control(set_point, curr_point):
    global last_position, last_set_point, timer_started, times, set_points, curr_points, start_time

    if not timer_started:
        start_time = time.time()
        timer_started = True

    position_err = set_point - curr_point
    derivative_err = (last_position - curr_point)/(sampling_rate_ms/1000)

    last_position = curr_point
    last_set_point = set_point

    effort = position_err*Kp + derivative_err*Kd

    times.append(time.time())
    set_points.append(set_point)
    curr_points.append(curr_point)

    write_effort(effort, sampling_rate_ms/1000.0)

    if time.time() > (start_time + record_time_interval):
        rospy.loginfo("Recorded data")
        record_data(times, set_points, curr_points)
        reset_timer()


def callback(msg):
    d3 = get_position('d3')
    do_pd_control(msg.data, d3)

def main():

    rospy.init_node("position_controller")
    sub = rospy.Subscriber("/scara/set_point", Float64, callback)
    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        do_pd_control(last_set_point, get_position('d3'))
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass