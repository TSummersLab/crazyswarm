#!/usr/bin/env python
'''
Script to test the fullState command using multiple drones. The script moves each drone to init_pos + change_array

Author:
Sleiman Safaoui
GitHub:
The-SS
Email:
snsafaoui@gmail.com

Date: Oct 15, 2018
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
from crazyflie_helper_functions.core_helpers import *
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
import uav_trajectory

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

def main():

    kp_pos = 2.0
    max_p_change = 0.2
    change_array = np.array([0.5, 0.0, 0.5])


    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cf_goals = []
    for cntr, cf in enumerate(allcfs.crazyflies):
        cf_goals.append(cf.initialPosition)
        cf_goals[cntr] += change_array

    # wait until RB button is pressed to hover the cfs[0.0, 0.0, 0.0]
    print("press RB button to continue")
    swarm.input.waitUntilButtonPressed()

    # hover the cfs at an initial height
    allcfs.takeoff(0.5, 3.0)
    timeHelper.sleep(3.0)
    # for cf in allcfs.crazyflies:
    #     cf.takeoff(targetHeight = 0.5, duration = 3.0)
    #     timeHelper.sleep(3.0)

    print("press RB button to stop")

    while not rospy.is_shutdown():
        for cntr, cf in enumerate(allcfs.crazyflies):
            des_pos = cf_goals[cntr]
            cur_pos = cf.position()
            pos_ctrl = kp_pos * (des_pos - cur_pos)
            if np.linalg.norm(pos_ctrl) > max_p_change:
                pos_ctrl = pos_ctrl/np.linalg.norm(pos_ctrl)*max_p_change
            pos_cmd = pos_ctrl + cur_pos

            pose = np.zeros(7)
            twist = np.zeros(6)
            acc = np.zeros(3)
            pose[0:3] = pos_cmd
            pose[3:8] = np.array([0,0,0,1])

            cf.cmdFullState(pose, twist, acc)

        # Check if RB button has been pressed
        button_pressed = swarm.input.checkIfButtonIsPressed()

        if button_pressed == True:
            break

    done = fullStateLandAllCfs(swarm)

if __name__ == "__main__":

    main()
