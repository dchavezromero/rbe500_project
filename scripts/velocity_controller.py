#!/usr/bin/env python
import rospy
import csv
import time
from gazebo_msgs.srv import *
from rbe500_project.msg import ee_velocities 
from rbe500_project.srv import EEToJointVels,EEToJointVelsRequest

joint_names = 'theta1', 'theta2', 'd3'
#20, 20
Kp_vals = [20, 20, 20]
Kd_vals = [0, 0, 0]

# 10Hz frequency
ros_rate = 10.0

# Loop execution rate in seconds (1/freq)
sampling_rate = (1/ros_rate) 
last_positions= [0.0, 0.0, 0.0]
last_velocities= [0.0, 0.0, 0.0]
last_ref_velocities = [0.0, 0.0, 0.0]

# Record data every 15 seconds
record_time_interval = 15

start_time = 0
timer_started = False

times = []
ref_vels_theta1 = []
ref_vels_theta2 = []
ref_vels_d3 = []

curr_vels_theta1 = []
curr_vels_theta2 = []
curr_vels_d3 = []

def reset_timer():
    global times, ref_vels_theta1, ref_vels_theta2, ref_vels_d3, curr_vels_theta1, curr_vels_theta2, curr_vels_d3, timer_started

    times = []
    ref_vels_theta1 = []
    ref_vels_theta2 = []
    ref_vels_d3 = []

    curr_vels_theta1 = []
    curr_vels_theta2 = []
    curr_vels_d3 = []

    timer_started = False

def record_data(times, ref_vels_theta1, ref_vels_theta2, ref_vels_d3, curr_vels_theta1, curr_vels_theta2, curr_vels_d3):

    print(len(times))
    print(len(ref_vels_theta1))
    print(len(ref_vels_theta2))
    print(len(ref_vels_d3))
    print(len(curr_vels_theta1))
    print(len(curr_vels_theta2))
    print(len(curr_vels_d3))

    with open('velocity_controller_data.csv', 'wb') as csv_file:

        writer = csv.writer(csv_file, delimiter=',')

        for i in range(len(times)-1):
            writer.writerow((times[i], ref_vels_theta1[i], curr_vels_theta1[i], ref_vels_theta2[i], curr_vels_theta2[i], ref_vels_d3[i], curr_vels_d3[i]))


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

def do_pd_control(ref_vel, curr_vel, joint_name):
    global last_velocities, last_ref_velocities, timer_started, times, ref_vels_theta1, ref_vels_theta2, ref_vels_d3, curr_vels_theta1, curr_vels_theta2, curr_vels_d3, start_time


    #print(ref_vel)

    # Start timer for recording data
    if not timer_started:
        start_time = time.time()
        timer_started = True

    # Calculate errors
    position_err = ref_vel - curr_vel

    if (joint_name is 'theta1'):

        derivative_err = (last_velocities[0] - curr_vel)/sampling_rate

        # Set reference vals
        last_velocities[0] = curr_vel
        last_ref_velocities[0] = ref_vel

        # PD output
        effort = position_err*Kp_vals[0] + derivative_err*Kd_vals[0]
        # Append data entries to lists
        ref_vels_theta1.append(ref_vel)
        curr_vels_theta1.append(curr_vel)

        # Append data entries to lists
        times.append(time.time())

    elif (joint_name is 'theta2'):

        derivative_err = (last_velocities[1] - curr_vel)/sampling_rate

        # Set reference vals
        last_velocities[1] = curr_vel
        last_ref_velocities[1] = ref_vel

        # PD output
        effort = position_err*Kp_vals[1] + derivative_err*Kd_vals[1]
        # Append data entries to lists
        ref_vels_theta2.append(ref_vel)
        curr_vels_theta2.append(curr_vel)

    elif (joint_name is 'd3'):

        derivative_err = (last_velocities[2] - curr_vel)/sampling_rate

        # Set reference vals
        last_velocities[2] = curr_vel
        last_ref_velocities[2] = ref_vel

        # PD output
        effort = position_err*Kp_vals[2] + derivative_err*Kd_vals[2]
        # Append data entries to lists
        ref_vels_d3.append(ref_vel)
        curr_vels_d3.append(curr_vel)



    # Write effort to joint
    write_effort(effort, sampling_rate, joint_name)

    # If 15 secs have passed, record the data to a CSV file and reset timer params
    if time.time() > (start_time + record_time_interval):
        rospy.loginfo("Recorded data")
        record_data(times, ref_vels_theta1, ref_vels_theta2, ref_vels_d3, curr_vels_theta1, curr_vels_theta2, curr_vels_d3)
        reset_timer()


def get_joint_vels(curr_ee_vels):
    rospy.wait_for_service('/ee_vels_to_joint_vels')
    try:
        conv_call = rospy.ServiceProxy('/ee_vels_to_joint_vels', EEToJointVels)
        joint_req = EEToJointVelsRequest()
        joint_req.ee_vels = curr_ee_vels
        joint_vel_data = conv_call(joint_req)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return joint_vel_data.joint_vels.theta1_velocity, joint_vel_data.joint_vels.theta2_velocity, joint_vel_data.joint_vels.d3_velocity

def get_curr_joint_vels():
    theta1 = get_position(joint_names[0])
    theta2 = get_position(joint_names[1])
    d3 = get_position(joint_names[2])

    theta1_vel = (last_positions[0] - theta1)/sampling_rate
    theta2_vel = (last_positions[1] - theta2)/sampling_rate
    d3_vel = (last_positions[2] - d3)/sampling_rate

    last_positions[0] = theta1
    last_positions[1] = theta2
    last_positions[2] = d3
    
    return theta1_vel, theta2_vel, d3_vel

def callback(msg):
    joint_vels = get_joint_vels(msg)

    theta1_vel = joint_vels[0]
    theta2_vel = joint_vels[1]
    d3_vel = joint_vels[2]

    curr_joint_vels = get_curr_joint_vels()

    do_pd_control(theta1_vel, curr_joint_vels[0], joint_names[0])
    do_pd_control(theta2_vel, curr_joint_vels[1], joint_names[1])
    do_pd_control(d3_vel, curr_joint_vels[2], joint_names[2])

def main():
    rospy.init_node("velocity_controller")

    sub = rospy.Subscriber("/scara/ref_velocities", ee_velocities, callback)
    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        curr_joint_vels = get_curr_joint_vels()

        do_pd_control(last_ref_velocities[0], curr_joint_vels[0], joint_names[0]) # maintain theta1 vels
        do_pd_control(last_ref_velocities[1], curr_joint_vels[1], joint_names[1]) # maintain theta2 vels
        do_pd_control(last_ref_velocities[2], curr_joint_vels[2], joint_names[2]) # maintain d3 vels
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass