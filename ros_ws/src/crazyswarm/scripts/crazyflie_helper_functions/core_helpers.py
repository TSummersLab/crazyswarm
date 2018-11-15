#!/usr/bin/env python
"""
Core helper module for the crazyswarm package. This module is designed to provide functions that help operate the crazyflie drones.

This file (core_helper.py) provides general functions/classes to be used with crazyswarm.

Please read the description of each function for its description

Author:
Sleiman Safaoui
GitHub:
The-SS
Email:
snsafaoui@gmail.com


Date:
Oct 9, 2018
"""

import copy
import numpy as np
import sys
import math

def fullStateLandAllCfs(swarm, landing_heights=[]):
    '''
    Lands all crazyflies by sending full_state commands to the drones.
    Input:
            swarm: object of type Crazyswarm() #swarm = Crazyswarm()
            landing_heights: 1D list/array containing the heights for the individual drones to land at. If empty or not provided the drones will land on global z = 0 plane
    Output:
            success: bool indicating if the sequence was successful

    If landing takes more than 60 seconds, sequence is stopped
    '''

    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # find the current position of all drones (they will land at that x-y position)
    init_pos = [] # current position
    des_pos = [] # desired landing position
    for cntr, cf in enumerate(allcfs.crazyflies):
        cf_cur_pos = cf.position()
        init_pos.append(cf_cur_pos)
        cf_des_pos = copy.copy(cf_cur_pos)
        if landing_heights == []:
            cf_des_pos[-1] = 0.05
        else:
            try:
                cf_des_pos[-1] = landing_heights[cntr]
            except: # if not enough landing heights are provided, assume global z = 0 as landing height
                e = sys.exc_info()[0]
                print('Error ', e)
                print("Will assume global z = 0 as landing height for cf ", cntr)
                cf_des_pos[-1] = 0.05
        des_pos.append(cf_des_pos)

    # Initialize variables for landing
    num_agents = len(des_pos)
    kp = 1.0 # gain for position control signal
    d_max = 0.05 # maximum distance to travel in a step
    cf_landed = np.array(np.zeros(num_agents), dtype=bool) # indicates which drones have landed
    max_wait_time = 60 # 60 seconds before landing sequence terminates
    sleep_time = 0.001 # sleep time

    # land drones
    t_start = timeHelper.time()
    t_now = timeHelper.time()
    while ((cf_landed.all() == False) & (t_now - t_start <= max_wait_time)):
        for cntr, cf in enumerate(allcfs.crazyflies):
            if cf_landed[cntr]: # cf has landed
                cf.cmdStop() # stop motors
            else:
                cf_cur_pos = cf.position()
                cf_des_pos = des_pos[cntr]
                if (cf_cur_pos[-1] <= (cf_des_pos[-1]+0.1)): # cf height close to landing height
                    cf.cmdStop() # stop motors
                    cf_landed[cntr] = True # set cf_landed to True
                else: # cf still descend
                    # find state for descend
                    pos_ctrl = kp * (cf_des_pos - cf_cur_pos)
                    if np.linalg.norm(pos_ctrl) > d_max:
                        pos_ctrl = pos_ctrl/np.linalg.norm(pos_ctrl)*d_max
                    pos_cmd = pos_ctrl + cf_cur_pos
                    pose = np.zeros(7)
                    twist = np.zeros(6)
                    acc = np.zeros(3)
                    pose[0:3] = pos_cmd
                    pose[3:8] = np.array([0,0,0,1])

                    cf.cmdFullState(pose, twist, acc) # send state to drone
        timeHelper.sleep(sleep_time) # sleep / pause
        t_now = timeHelper.time() # update time

    for cntr, cf in enumerate(allcfs.crazyflies):
        cf.cmdStop() # stop
        timeHelper.sleep(0.05) # sleep
        cf.land(des_pos[cntr][-1], 0.0, 0.0) # land
    timeHelper.sleep(max(num_agents * 0.1, 3.0)) # sleep
    return True
