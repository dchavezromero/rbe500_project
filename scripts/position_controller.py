#!/usr/bin/env python
import rospy
import csv
import time
from gazebo_msgs.srv import *
from rbe500_project.msg import joint_velocities, joint_angles 

joint_names = 'theta1', 'theta2', 'd3'

Kp_vals = [25, 70, 40]
Kd_vals = [25, 10, 20]

# 10Hz frequency
ros_rate = 10.0

# Loop execution rate in seconds (1/freq)
sampling_rate = (1/ros_rate) 
last_positions = [0.0, 0.0, 0.0]
last_set_points = [0.0, 0.0, 0.0]

# Record data every 15 seconds
record_time_interval = 15

start_time = 0
timer_started = False
times = []
set_points = []
curr_points = []

joint_vels = joint_velocities()

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


def write_effort(effort, duration_sec, joint_name):
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    try:
        joint_call = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        joint_req = ApplyJointEffortRequest()
        joint_req.joint_name = joint_name
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

def do_pd_control(set_point, curr_point, joint_name):
    global last_positions, last_set_points, timer_started, times, set_points, curr_points, start_time

    # Start timer for recording data
    if not timer_started:
        start_time = time.time()
        timer_started = True

    # Calculate errors
    position_err = set_point - curr_point

    if (joint_name is 'theta1'):

        derivative_err = (last_positions[0] - curr_point)/sampling_rate
        joint_vels.theta1_velocity = derivative_err #rad/sec

        # Set reference vals
        last_positions[0] = curr_point
        last_set_points[0] = set_point

        # PD output
        effort = position_err*Kp_vals[0] + derivative_err*Kd_vals[0]
    elif (joint_name is 'theta2'):

        derivative_err = (last_positions[1] - curr_point)/sampling_rate
        joint_vels.theta2_velocity = derivative_err

        # Set reference vals
        last_positions[1] = curr_point
        last_set_points[1] = set_point

        # PD output
        effort = position_err*Kp_vals[1] + derivative_err*Kd_vals[1]
    elif (joint_name is 'd3'):

        derivative_err = (last_positions[2] - curr_point)/sampling_rate
        joint_vels.d3_velocity = derivative_err

        # Set reference vals
        last_positions[2] = curr_point
        last_set_points[2] = set_point

        # PD output
        effort = position_err*Kp_vals[2] + derivative_err*Kd_vals[2]

    # Append data entries to lists
    times.append(time.time())
    set_points.append(set_point)
    curr_points.append(curr_point)

    # Write effort to joint
    write_effort(effort, sampling_rate, joint_name)

    # If 15 secs have passed, record the data to a CSV file and reset timer params
    if time.time() > (start_time + record_time_interval):
        rospy.loginfo("Recorded data")
        record_data(times, set_points, curr_points)
        reset_timer()


def callback(msg):
    theta1 = get_position(joint_names[0])
    theta2 = get_position(joint_names[1])
    d3 = get_position(joint_names[2])

    do_pd_control(msg.theta1, theta1, joint_names[0])
    do_pd_control(msg.theta2, theta2, joint_names[1])
    do_pd_control(msg.d3, d3, joint_names[2])

def main():
    rospy.init_node("position_controller")

    # Publisher for joint velocities
    pub_velocities = rospy.Publisher("/scara/joint_velocities", joint_velocities, queue_size=1)
    sub = rospy.Subscriber("/scara/set_points", joint_angles, callback)
    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        do_pd_control(last_set_points[0], get_position(joint_names[0]), joint_names[0]) # maintain theta1 position
        do_pd_control(last_set_points[1], get_position(joint_names[1]), joint_names[1]) # maintain theta2 position
        do_pd_control(last_set_points[2], get_position(joint_names[2]), joint_names[2]) # maintain d3 position

        pub_velocities.publish(joint_vels)
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass