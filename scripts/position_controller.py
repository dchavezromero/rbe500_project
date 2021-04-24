#!/usr/bin/env python
import rospy
import csv
import os
import time
from gazebo_msgs.srv import *
from std_msgs.msg import Float64

Kp = 40
Kd = 20
joint_name = 'd3'

# 10Hz frequency
ros_rate = 10.0

# Loop execution rate in seconds (1/freq)
sampling_rate = (1/ros_rate) 
last_position = 0.0
last_set_point = 0.0

# Record data every 15 seconds
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
        joint_req.duration.nsecs = duration_sec * 100000000 #Convert sampling rate to nanoseconds
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

    # Start timer for recording data
    if not timer_started:
        start_time = time.time()
        timer_started = True

    # Calculate errors
    position_err = set_point - curr_point
    derivative_err = (last_position - curr_point)/sampling_rate

    # Set reference vals
    last_position = curr_point
    last_set_point = set_point

    # PD output
    effort = position_err*Kp + derivative_err*Kd

    # Append data entries to lists
    times.append(time.time())
    set_points.append(set_point)
    curr_points.append(curr_point)

    # Write effort to d3 joint
    write_effort(effort, sampling_rate)

    # If 15 secs have passed, record the data to a CSV file and reset timer params
    if time.time() > (start_time + record_time_interval):
        rospy.loginfo("Recorded data")
        record_data(times, set_points, curr_points)
        reset_timer()


def callback(msg):
    d3 = get_position(joint_name)
    do_pd_control(msg.data, d3)

def main():

    rospy.init_node("position_controller")
    sub = rospy.Subscriber("/scara/set_point", Float64, callback)
    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        do_pd_control(last_set_point, get_position(joint_name))
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass