#!/usr/bin/env python
'''
We test the cmdFullStateSetpoint functionality using the cmdFullState function in crazyflie.py. The test tries to control a single drone (the first cf) to send it to a desired position.

Author:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
Github:
The-SS

Date:
Sept 25, 2018
'''


# python
import argparse
import numpy as np
import csv
import datetime
import time
import copy
import math

# ROS
import rospy

# Crazyswarm
from pycrazyswarm import *
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
import uav_trajectory


def store_data_csv(file_name, data, msg):
    '''
    Writes data to a file called file_name.csv (overwrites previous data)

    input:
            msg: string containing a message to be printed on the first line only
            file_name: string indicating file name
            data: list containing the data
    output:
            True if successful, False if not successful

    '''
    if file_name == "":
        print('Invalid file name')
        return False

    full_file_name = file_name + '.csv'

    with open(full_file_name, 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=",")
        csv_writer.writerow([msg])
        for i in range(len(data)):
            csv_writer.writerow(data[i])

    return True

def quaternion_to_euler_angles(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return np.array([X, Y, Z])

def main(args):
    '''
    Params and Gains
    '''
    kp_ctrl = 1.0 # Kp gain for ctrl vel signal
    vel_max = 0.1 # maximum velocity along a direction (x,y)
    vel_min = -vel_max # minimum velocity along a direction (x,y)
    d_max = 0.05 # max distance command to travel along the +z direction
    d_min = - d_max # min distance command to travel along the -z direction
    d_scale = 0.1 # scale used to change vertical velocity to vertical displacement (equivalent to time estimate of an iteration)
    kp_yaw = 0.15 # kp gain for yaw rotation
    des_yaw = 0.0 # desired yaw angle

    itr_len = 1.0 # iteration length for constructing a polynomial
    sleep_time = 0.01 # time to sleep before moving on to the next iteration

    # des_pos = np.array([1.0, -1.5, 0.75]) # goal destination
    # des_pos = np.array([-1.0, 1.5, 0.75]) # goal destination
    des_pos = np.array([0.0, 0.0, 0.5]) # goal destination

    kp_pos = 1.0
    kp_orient = 0.01
    kp_vel = 0.01
    max_p_change = 0.1



    # Create Crazyswarm object
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf  = allcfs.crazyflies[0]

    # wait until RB button is pressed to hover the cfs[0.0, 0.0, 0.0]
    print("press RB button to continue")
    swarm.input.waitUntilButtonPressed()

    # hover the cfs at an initial height
    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    timeHelper.sleep(3.0)

    cur_pos = np.array(cf.position())
    t_now = time.time()
    cur_rot_quat = np.array(cf.rotation())
    cur_rot_euler = quaternion_to_euler_angles(cur_rot_quat)
    cur_vel = np.array([0,0,0])

    #############################################################
    #############################################################
    #############################################################

    # traj = uav_trajectory.Trajectory()
    des_pos_reset = False

    print("press RB button to stop")


    while not rospy.is_shutdown():
        print('heading to destination')
        # old data
        old_pos = cur_pos
        old_rot_quat = cur_rot_quat
        t_old = t_now
        old_vel = cur_vel


        # current time and pose
        t_now = time.time()
        cur_pos = np.array(cf.position())
        cur_rot_quat = np.array(cf.rotation())
        cur_vel = np.linalg.norm((cur_pos - old_pos))/(t_now - t_old)
        # cur_rot_euler = quaternion_to_euler_angles(cur_rot_quat)
        # yaw = cur_rot_euler[2]

        # initializing control variables
        pose = np.zeros(7)
        twist = np.zeros(6)
        acc = np.zeros(3)

        # # desired control
        pos_ctrl = kp_pos * (des_pos - cur_pos)
        if np.linalg.norm(pos_ctrl) > max_p_change:
            pos_ctrl = pos_ctrl/np.linalg.norm(pos_ctrl)*max_p_change
        pos_cmd = pos_ctrl + cur_pos
        orient_ctrl = kp_orient * (np.array([0, 0, 0, 1])- cur_rot_quat)
        velocity_ctrl = kp_vel * (np.array([0,0,0]) - cur_vel)

        # assigning control
        # pose[0:3] = pos_ctrl
        # pose[3:8] = orient_ctrl
        # twist[0:3] = velocity_ctrl

        pose[0:3] = pos_cmd
        pose[3:8] = np.array([0,0,0,1])

        print("Full State:")
        print("pose: ", pose)
        print("twist: ", twist)
        print("acc: ", acc)

        cf.cmdFullState(pose, twist, acc)
        timeHelper.sleep(sleep_time)

        # Check if RB button has been pressed
        button_pressed = swarm.input.checkIfButtonIsPressed()


        if button_pressed == True:
        # if (np.linalg.norm(cur_pos-des_pos) < 0.1) & (des_pos_reset == False):
            des_pos = np.array([cur_pos[0], cur_pos[1], 0.05])
            des_pos_reset = True

        if (des_pos_reset == True) & (button_pressed == True) & (cur_pos[2]<0.1):

            cf.cmdStop()
            break

    cf.land(0.05, 0.5, 0.0)
    timeHelper.sleep(2.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="Run using simulation", action="store_true")
    args = parser.parse_args()
    # try:
    main(args)
    # except:
    #     print('Error occured')
