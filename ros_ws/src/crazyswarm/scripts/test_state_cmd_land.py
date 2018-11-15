#!/usr/bin/env python
'''
Script to test the fullStateLandAllCfs function in crazyflie_helper_functions/core_helpers

Author:
Sleiman Safaoui
GitHub:
The-SS
Email:
snsafaoui@gmail.com

Date: Oct 10, 2018
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
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf  = allcfs.crazyflies

    # wait until RB button is pressed to hover the cfs[0.0, 0.0, 0.0]
    print("press RB button to continue")
    swarm.input.waitUntilButtonPressed()

    # hover the cfs at an initial height
    for cf in allcfs.crazyflies:
        cf.takeoff(targetHeight = 0.5, duration = 3.0)
        timeHelper.sleep(3.0)

    print("press RB button to continue")
    swarm.input.waitUntilButtonPressed()

    done = fullStateLandAllCfs(swarm)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="Run using simulation", action="store_true")
    args = parser.parse_args()
    main(args)
