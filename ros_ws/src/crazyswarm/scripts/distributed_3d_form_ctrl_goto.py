#!/usr/bin/env python
'''
We apply distributed 3D formation control law to Crazyflies modeled as single integrators and issue commands using the Crazyswarm "hover" function to achieve formation.

Required edits are to be done in main.

Author:
Sleiman Safaoui
Github:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Theory:
Kaveh Fathian
Github:
KavehFathian
Email:
kavehfathian@gmail.com

Sept 10, 2018
'''
# python
import argparse
import numpy as np
import csv
import datetime
import time
import copy
import os

# ROS
import rospy

# Crazyswarm
from pycrazyswarm import *
from crazyflie_helper_functions.distributed_collision_avoidance import *
from crazyflie_helper_functions.safety_net import *
from crazyflie_helper_functions.rotation_conversion import Q2R

# Formation control
from distributed_formation_control.formations import load_shapes

# Matlab engine (keep as last import to avoid weird ros bug)
import matlab.engine

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

# for a fast implementation, kp_ctrl = 55; d_sat = 0.115; itr_len =  0.01; sleep = 2*itr_len
# for a slow implementation, kp_ctrl = 0.01, d_sat = 1; itr_len =  4.5; sleep = itr_len - 1.0

def main(args):
    '''
    Params and Gains
    '''
    # Formation Control Gains
    # des_form_scale = 0.8  # scale for desired formation inter-agent distances
    k_dist = 0.3#0.5 #2.5 # 0.5 # multiplier for distance control gain obtained from calculating the gains matrix (A) in matlab
    kp_ctrl = 0.3#20 #100.0 #0.3 # Proportional control gain (used when discritizing velocity control)
    d_sat = 0.07 #0.05 # 0.07  # saturation distance (max distance that can be traveled in the given time~sleep time)
    iteration_length = 1.# length of an iteration in seconds (for discritizing the trajectory)
    sleep_time = 0.05 #0.9 # 0.05

    # Collision Avoidance Paramerters
    dcoll = 0.5 # radius of ball around agent to look for Neighbors
    rcoll = 0.4 # radius of ball around neighbors to avoid

    # Safety_net Params
    min_x = -1.5
    max_x = 1.5
    min_y = -2.0
    max_y = +2.0
    min_z = +0.30
    max_z = 1.45
    cf_safe_ball_radius = 0.075

    # Saving Data Params
    save_data = True

    # Leader
    leader_index = 3 # 0-based indexing
    leader_pos_des = np.array([0.0, 0.0, 0.40]) # desired position of leader

    # Formations Control Params
    formation_control_type = "fixed" # "free" or "fixed"
    current_path = os.getcwd()
    path_to_matlab_files = os.path.abspath('distributed_formation_control/matlab')
    # path_to_matlab_files = '/home/tylersummers/ssh_cs_ws_3/crazyswarm/ros_ws/src/crazyswarm/scripts/3D_formation_control_matlab/'
    Dd = [] # desired distance (initialized if formation is fixed scale)

    # Collision avoidance type
    col_avoid_type = "2D" # "2D" or "3D"

    # Formation Control Shape
    form_ctrl_shape = 'sq_base_pyramid'
    des_form_scale, dcoll, rcoll, num_agents, q_des_list, adj_array = load_shapes(form_ctrl_shape)

    q_des_list = map(list, zip(*q_des_list))
    q_des_array = np.array(q_des_list) * des_form_scale # scaled array version of q_des_list. Array with shape: 3 x num_agents
    adj_list = copy.copy(adj_array).tolist() # List version of adj_array. List with shape: num_agents x num_agents


    # Permutation matrix for swapping agents
    E = np.eye(num_agents)


    '''
    Initializations and checks
    '''

    print('Initializations')
	# Safety Net Initialization and Check
    net = SafetyNet()
    set_safety_net_params_check = net.set_params(min_x, max_x, min_y, max_y, min_z, max_z, cf_safe_ball_radius, num_agents)
    if set_safety_net_params_check == False: # could not set params
        print("Invalid safety net parameters. ABORTING!")
    	return # stop the code

    # Collision Avoidance Initializations
    ca = ColAvoid3D()
    ca.hedSamples = 50 # reduce the size of the sphere point cloud to consider
    ca.update_static_params(num_agents, dcoll, rcoll)
    if col_avoid_type == "2D":
        ca.update_sphere_samples_2D()
    elif col_avoid_type == "3D":
        ca.update_sphere_samples()
    else:
        print("Invalid collision avoidance type. ABORTING!")


    # Formation Control Type Check
    if ((formation_control_type != "free") & (formation_control_type != "fixed")):
        print('Invalid formation control type. Use "free" or "fixed" only.')
        return

    # Saving Data Initializations TODO: EDIT THIS LIST
    if save_data == True:
        '''
        variables to store data in
        they all have the following format:
        [ [itr1] [itr2] ... [itrN] ]
        where [itrX] is a list containing the corresponding data as a 1D array:
        e.g.: [x1 y1 z1 x2 y2 x2 ... xN yN zN]
        	  [t1 t2 ... tN]
        '''
        experiment_time = datetime.datetime.now() # get experiment time
        timestamp = [] # list of all timestamps when position is fetched
        q_all = [] # list of all positions fetched
        dq_all = [] # list of all dq values calculated
        ctrl_all = [] # list of all control signals calculated
        q_des_calc = [] # list of all desired positions calculated
        q_des_safe = [] # list of all safe desired positions issued to the crazyflies
        safety_net_data_all = [] # list of results obtained from applying the safety net
        E_all = [] # list of rows of the permutation matrix E
        stopFlag_all = [] # list of flags indicating whether a drone should have stopped or not
        stopFlag_old_all = [] # list of flags indicating whether a drone should have stopped or not (before resolving grid-lock)
        colIdx_all = [] # list all all collision indicies

    # Desired Distance (Dd) Initialization
    if formation_control_type == 'fixed':
        Dd = np.zeros([num_agents, num_agents], dtype=float)
        for agent in range(num_agents):
            for neighbor in range(num_agents):
                Dd[agent, neighbor] = np.linalg.norm(q_des_array[:,agent] - q_des_array[:,neighbor])
    print('Desired Distances:\n', Dd)

    # Calculate Gain Matrix (A)
    print("Connecting to MATLAB ...")
    matlab_engine = matlab.engine.start_matlab()
    print("Connected to MATLAB successfully!")
    matlab_engine.addpath(path_to_matlab_files + '/Helpers')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist/structures')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist/lib')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist/functions')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist/commands')
    matlab_engine.addpath(path_to_matlab_files + '/cvx_redist/builtins')
    print("Finding gain Matrix in MATLAB")
    A, Kd = matlab_engine.FindGains3D_Ver1_0(matlab.double(q_des_list), matlab.double(adj_list), nargout=2)
    print("Gain matrix found")
    print("Disconnecting from MATLAB ...")
    matlab_engine.quit()
    print("Disconnected from MATLAB successfully!")
    print('gain matrix A')
    print(A)
    print(np.array(A).shape)

    k_dist = Kd*k_dist

    '''
    Controlling Robots
    '''
    # Create Crazyswarm object
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # wait until RB button is pressed to hover the cfs[0.0, 0.0, 0.0]
    print("press RB button to continue")
    swarm.input.waitUntilButtonPressed()

    # hover the cfs at an initial height
    allcfs.takeoff(targetHeight = 0.5, duration = 3.0)
    timeHelper.sleep(3.0)

    # Get current positon
    q_list = get_current_pos_list(allcfs.crazyflies) # list form with shape: num_agents x 3
    q_original = np.transpose(np.array(q_list, dtype=float)) # transpose array version of q_list. Shape: 3 x num_agents
    q_original_stack = np.hstack(np.transpose(q_original)) # array with positions stacked (x1,y1,z1, x2,y2,z2, ...). Shpae: 3*num_agents x 1

    # permute positions based on E
    E3 = np.kron(E,np.eye(3))
    q_stack = np.dot(E3, q_original_stack) # permuted array of positions of robots. Shape: 3*num_agents x 1
    q = np.transpose(q_stack.reshape([num_agents, 3]))# array with permuted positions of robots. Shape: 3 x num_agents


    # most recent positions with stopFlag of 0 (to hover at in case a new collision free position was not found):
    pos_for_stop_flag_on = []
    for i in range(num_agents):
        pos_for_stop_flag_on.append([]) # each entry would be the most recent safe position for that particular agent

    new_itr_time = time.time()


    while not rospy.is_shutdown():
        prev_itr_time = new_itr_time
        new_itr_time = time.time()
        # print('The whole iteration executed in this many seconds: ', new_itr_time - prev_itr_time)

        # Get current position of robots and current body frame rotations as quaternions
        q_list = get_current_pos_list(allcfs.crazyflies) # position list with shape: num_agents x 3
        rot_list = get_current_rot_list(allcfs.crazyflies) # rotation quatornians list with shape: num_agents x 4

        # Stack the positions
        q_original = np.transpose(np.array(q_list, dtype=float)) # transpose array version of q_list. Shape: 3 x num_agents
        q_original_stack = np.hstack(np.transpose(q_original)) # array with positions stacked (x1,y1,z1, x2,y2,z2, ...). Shpae: 3*num_agents x 1

        # Permute positions based on E
        E3 = np.kron(E,np.eye(3))
        q_stack = np.dot(E3, q_original_stack) # permuted array of positions of robots. Shape: 3*num_agents x 1
        q_all_world = np.transpose(q_stack.reshape([num_agents, 3]))# array with permuted positions of robots. Shape: 3 x num_agents


        # Stack the quaternions
        rot_original = np.transpose(np.array(rot_list, dtype=float)) # transpose array version of rot_original. Shape: 4 x num_agents
        rot_original_stack = np.hstack(np.transpose(rot_original)) # array with quaternions stacked (x1,y1,z1,w1, x2,y2,z2,w2, ...). Shpae: 4*num_agents x 1

        # Permute quaternions based on E
        E4 = np.kron(E,np.eye(4))
        rot_stack = np.dot(E4, rot_original_stack) # permuted array of quaternions of robots. Shape: 4*num_agents x 1
        rot_all_world = np.transpose(rot_stack.reshape([num_agents, 4]))# array with permuted quaternions of robots. Shape: 4 x num_agents

        # Now we have:
        # q_all_world: permuted positions of robots (relative to wold frame). Type: array. Shape: 3 x num_agents
        # rot_all_world: permuted body frame rotation quaternions of the robots (relative to wold frame). Type: array. Shape: 4 x num_agents
        # We will use them to do all necessary calculations


        # Find Rotation matrix from quaternions
        rot_mat_all_world = [] # rotation matrix list containing rotation matrices arrays. Shape: num_agents x (3x3)
        for agent in range(num_agents):
            rot_mat_all_world.append(Q2R(rot_all_world[:,agent]))


        '''
        Find the rotation and translation of every agent's neighbors in its own frame
        '''
        q_all_frames = np.zeros([num_agents, num_agents, 3]) # position of each agent in frames of all its neighbors (each row refers to an agent as the reference frame (elements in that row are positions relative to the agent with the row's index))
        dq_in_agent = np.zeros([num_agents,3]) # control velocity vector for each agent in its own reference frame
        dq_in_world = np.zeros([num_agents,3]) # control velocity vector for each agent in the world reference frame

        for agent in range(num_agents):
            H_agent_in_w = np.zeros([4,4]) # transformation of agent body frame relative to world frame
            R_agent_in_w = np.zeros([3,3]) # rotation of agent body frame relative to world frame

            # Populate H and R
            pos_agent_in_world = copy.copy(q_all_world[:,agent]) #position of the seleced agent in the world frame
            # quaternion_agent_in_world = rot_all_world[:,agent] #quaternion rotation of the seleced agent in the world frame
            rotation_agent_in_world = rot_mat_all_world[agent]
            R_agent_in_w = rotation_agent_in_world
            H_agent_in_w[0:3,0:3] = rotation_agent_in_world
            H_agent_in_w[0:3,3] = pos_agent_in_world
            H_agent_in_w[3,3] = 1.

            for neighbor in range(num_agents): # For each neighbor, find the position of the neighbor in the agent's body frame
                if neighbor == agent:
                    q_all_frames[agent, neighbor] = [0., 0., 0.] # the agent's relative postion to itself is zero
                else:
                    pos_neighbor_in_world = np.array(q_all_world[:,neighbor]) #position of the seleced neighbor in the world frame
                    # Find inverse of transformation of agent body frame relative to world frame (H_agent_in_w)
                    try:
                        H_w_in_agent = np.linalg.inv(H_agent_in_w) # inverse of H_agent_in_w
                    except:
                        H_w_in_agent = np.zeros([4,4])
                        H_w_in_agent[0:3, 0:3] = np.transpose(rotation_agent_in_world)
                        H_w_in_agent[0:3, 3] = -np.dot(np.transpose(rotation_agent_in_world),pos_agent_in_world)
                        H_w_in_agent[3, 3] = 1.
                    # Change the position of the neighbor to a homogeneous representation
                    pos_neighbor_in_world_homo = list(pos_neighbor_in_world)
                    pos_neighbor_in_world_homo.append(1.0)
                    pos_neighbor_in_world_homo = np.array(pos_neighbor_in_world_homo)
                    # Find the position of the neighbor in the agent's body frame
                    pos_homo = np.dot(H_w_in_agent, pos_neighbor_in_world_homo) # still homogeneous representation for position
                    q_all_frames[agent, neighbor] = pos_homo[0:3] # store only first three entries (x,y,z)

            dq_in_agent[agent] = np.array([0.0, 0.0, 0.0])
            # Find the free-scale control velocity vector for an agent using A, adj_array, and the local position of every neighbor in its own body frame (decentralized)
            for neighbor in range(num_agents):
                if adj_array[agent, neighbor] == 1: # if the two robots are neighbors, take it into account when calculating the control signal
                    Aij = np.array(A)[3*agent:3*agent+3, 3*neighbor:3*neighbor+3]
                    dq_in_agent[agent] += np.dot(Aij,np.array(q_all_frames[agent, neighbor, :]))
            print('Agent:', agent)
            print('u free scales: ', dq_in_agent[agent])


        	# Add the fixed-scale non-linear term to the control velocity vector
            if formation_control_type == "fixed":
                Dc = np.zeros([num_agents], dtype=float) # currnet inter-agent distance between the agent and its neighbors. Shape: num_agents x 1
                for neighbor in range(num_agents):
                    Dc[neighbor] = np.linalg.norm(np.array(q_all_frames[agent, neighbor]))
                    Ddiff = adj_array[agent, neighbor]*(Dc[neighbor]-Dd[agent, neighbor]) # element wise multiplication
                    F = k_dist * np.arctan(Ddiff)
                    F = F * q_all_frames[agent, neighbor]
                    dq_in_agent[agent] += F # add F to the previously calculated velocity control
            print('u fixed scale: ', dq_in_agent[agent])

            # Return the control signals to the world frame (sending commands is a centralized command)
            dq_in_world[agent] = np.dot(R_agent_in_w, dq_in_agent[agent])
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        # ve = np.zeros(num_agents)
        # for i in range(num_agents):
        #     ve[i] = i
        # veE = np.dot(E,ve)
        # leader_index_new = veE[int(leader_index)]
        # u_in_world[int(leader_index_new)] = 10*(leader_pos_des - pos_all_world[int(leader_index_new)])

        '''
        Apply Collision Avoidance
        '''
        # Apply Collision Avoidance algorithm to the control
        dq_in_world_transpose = np.transpose(dq_in_world) # change the matrix into a 3 x num_agents

        before = time.time()
        if col_avoid_type == "2D":
            dq_in_world_no_col, stopFlag, colIdx = ca.ColAvoid2D_Ver_1_2(dq_in_world_transpose, q_all_world) # 2D collision avoidance
        elif col_avoid_type == "3D":
            dq_in_world_no_col, stopFlag, colIdx = ca.ColAvoid3D_Ver_1_2(dq_in_world_transpose, q_all_world) # 3D collision avoidance
        # dq_in_world_no_col is 3 x num_agents

        after = time.time()
        # print('Collision Avoidance took this many seconds: ', after-before)

        dq_in_world_temp = np.transpose(np.array(dq_in_world_no_col)) # change control to a num_agents x 3

        dq_before_collision_avoidance = copy.copy(dq_in_world)
        dq_in_world = copy.copy(dq_in_world_temp) # overwrite the previous control value (num_agents x 3)

        '''
        Gridlock resolution
        '''
        stopFlag_old = copy.copy(stopFlag) # store stopFlag before gridlock resolution (it will be overwritten)
        E_new = copy.copy(E) # onlu update E at end of this iteration
        for i in range(num_agents): # for each agent
            if stopFlag[i]: # if agent has to stop due to collisions
                for j in colIdx[i]: # go through the list of indices of adjacent agents with which agent i can switch
                    if (stopFlag[j]): # if agent j is also stuck
                        if np.any(np.array(colIdx[j])==i): # if agent j can switch with agent i
                        # Swap agents i and j (send them to the goal of the other agent)
                            row = copy.copy(E_new[i,:])
                            E_new[i,:] = E_new[j,:]
                            E_new[j,:] = row
                            stopFlag[i] = False
                            stopFlag[j] = False
                            break

        '''
        applying control (discretize dq_in_world)
        '''

        ctrl = np.zeros([num_agents, 3])
        for agent in range(num_agents):
            ctrl[agent] = kp_ctrl * iteration_length * dq_in_world[agent] # discretize
            print('Agent: ', agent)
            print('Ctrl No SAT: ',ctrl[agent])
            if stopFlag_old[agent]: # if the stopFlag is on (agent must not move)
                ctrl[agent] *= 0. # make contol zero
                print('(stopFlag) ctrl: ', ctrl[agent])

            d_norm = np.linalg.norm(ctrl[agent]) # find length of control signal
            if d_norm > d_sat:
                ctrl[agent] = ctrl[agent]/d_norm * d_sat # saturate if needed
                print('Ctrl SAT: ', ctrl[agent])

        # un-permute the control (undo effect of E) to send the correct control signal to the correct robot
        # Stack the control
        ctrl_original_stack = np.hstack(ctrl) # array with positions stacked (x1,y1,z1, x2,y2,z2, ...). Shpae: 3*num_agents x 1

        # Undo Permutation of control based on E
        E3 = np.kron(E,np.eye(3))
        ctrl_stack = np.dot(np.transpose(E3), ctrl_original_stack) # un-permuted array of positions of robots. Shape: 3*num_agents x 1
        ctrl_agents = ctrl_stack.reshape([num_agents, 3])# array with un-permuted control for robots. Shape: num_agents x 3


        # Add control to current position
        goal_pos_array_T = [] # goal positions
        for agent in range(num_agents): # for each agent
            goal_pos = ctrl_agents[agent] + q_list[agent] # add distance control (3 x 1 vector) to current position (3 x 1 vector)
            goal_pos_array_T.append(list(goal_pos)) # add the goal to the list of goal positions


        '''
        Apply Safety Net (check if control is safe to apply (within flyable region and collision free))
        '''

        current_pos_list = np.transpose(q_all_world).tolist() # current positon list with shape: num_agents x 3
        goal_pos_list = goal_pos_array_T # goal position list with shape: num_agents x 3

        # Check if the control is safe to apply
        update_pos_check = net.update_pos(current_pos_list, goal_pos_list)
        # print('current_pos_list', current_pos_list)
        # print('goal_pos_list',goal_pos_list)
        if update_pos_check == False: # could not update positions
            return # something is wrong. this should not happpen

        safety_check, agents_outside_flyable, intersecting_agents = net.find_des_pos_safe() # find safe desired positions
        if safety_check == True:
            safe_goal_pos_list = net.get_des_pos_safe() # list of safe goals. Shape: num_agents x 3
        else:
            safe_goal_pos_list = current_pos_list # list of safe goals. Shape: num_agents x 3


        '''
        Issue commands to drones
        '''

        # send control commands (hover positions) to crazyflies
        hover_timer = time.time()
        i = 0
        for cntr, cf in enumerate(allcfs.crazyflies):
            cf.goTo(goal = np.array(safe_goal_pos_list[cntr]), yaw = 0, duration = iteration_length, groupMask = 0)
        end_hover_timer = time.time()
        # print('Sent hover command in: ', end_hover_timer -  hover_timer)
        timeHelper.sleep(sleep_time) #
        end_hover_timer = time.time()
        # print('Hover + sleep complete in this many seconds: ', end_hover_timer -  hover_timer)

        # calculate current inter-agent distances
        inter_agent_dist = np.zeros([num_agents, num_agents], dtype=float) # (shpae: um_agents x num_agents
        for agent in range(num_agents):
            for neighbor in range(num_agents):
                inter_agent_dist[agent,neighbor] = np.linalg.norm(q_all_world[:,agent] - q_all_world[:,neighbor])
        # print("inter_agent_dist")
        # print(inter_agent_dist)

		# if fixed scale, calculate error in desired inter-agent distances
        if formation_control_type == 'fixed':
            Dd_error = Dd-inter_agent_dist
            # print('Error in desired distance Dd-Dc: ')
            # print(Dd_error)


        # if data is to be stored, append the list of data
        if save_data == True:
            '''
            aggregate data to save in temporary lists
            '''
            timestamp_itr = [timeHelper.time()]
            timestamp.append(timestamp_itr)

            q_all_itr = []
            for i in range(num_agents):
                q_all_itr.extend(np.transpose(q_all_world[:,i]).tolist())
            q_all.append(q_all_itr)

            dq_all_itr = []
            for i in range(num_agents):
                dq_all_itr.extend(dq_in_world[i].tolist())
            dq_all.append(dq_all_itr)

            ctrl_all_itr = []
            for i in range(num_agents):
                ctrl_all_itr.extend(ctrl_agents[i].tolist())
            ctrl_all.append(ctrl_all_itr)

            q_des_calc_itr = []
            for i in range(num_agents):
                q_des_calc_itr.extend(goal_pos_list[i])
            q_des_calc.append(q_des_calc_itr)

            q_des_safe_itr = []
            for i in range(num_agents):
                q_des_safe_itr.extend(safe_goal_pos_list[i])
            q_des_safe.append(q_des_safe_itr)

            safety_net_data_itr = []
            for i in range(len(agents_outside_flyable)):
                safety_net_data_itr.append(agents_outside_flyable[i])
                safety_net_data_itr.append(intersecting_agents[i])
            safety_net_data_all.append(safety_net_data_itr)

            E_itr = []
            for i in range(num_agents):
                E_itr.extend(E[i,:])
            E_all.append(E_itr)

            stopFlag = list(np.array(stopFlag, dtype = int))
            stopFlag_all.append(stopFlag)

            stopFlag_old = list(np.array(stopFlag_old, dtype = int))
            stopFlag_old_all.append(stopFlag_old)

            for i in range(num_agents):
                colIdx_all.append(colIdx[i])

        # Check if RB button has been pressed
        button_pressed = swarm.input.checkIfButtonIsPressed()
        if button_pressed == True:
            break # if pressed, break out of the while loop


        E = copy.copy(E_new) # update E to the newly found permuation

    # land the crazyflies
    try:
        allcfs.land(targetHeight = 0.06, duration = 3.0)
        timeHelper.sleep(3.0)
    except:
        pass

	# Save data
    if save_data == True:
        print('Saving data')
        str_time = experiment_time.strftime("%B_%d_%Y_at_%H_hr_%M_min_%S_sec")
        timestamp_msg = "Timestamps for data. Row number here corresponds to the row of the same number in the other data files"
        pos_msg = "Position of the robots at each timestamp. Each row has the following format: x1,y1,z1,x2,y2,z2,...xN,yN,zN where N is the number of robots"
        ctrl_vel_msg = "Velocity control calculated for each robot at each timestamp. Each row has the following format: Vx1,Vy1,Vz1,Vx2,Vy2,Vz2,...VxN,VyN,VzN where N is the number of robots"
        ctrl_pos_msg = "Position control calculated for each robot at each timestamp. Each row has the following format: x1,y1,z1,x2,y2,z2,...xN,yN,zN where N is the number of robots"
        des_pos_msg = "Desired position for the robots at each timestamp. Each row has the following format: x1,y1,z1,x2,y2,z2,...xN,yN,zN where N is the number of robots"
        safe_des_pos_msg = "Safe desired positions for the robots (command sent) at each timestamp. Each row has the following format: x1,y1,z1,x2,y2,z2,...xN,yN,zN where N is the number of robots"
        safety_net_msg = "Safety net results at each time_stamp. Each row has 2*num_agents elements. The data is formatted as follows: outside_check1, intersection_check1, outside_check2, intersection_check2, ... outside_checkN, intersection_checkN where N is the number of robots. outside_check == 1 --> agents outside flyable region. outside_check == 0 --> agents inside flyable region. intersection_check == 1 --> agent intersects with another agent. intersection_check == 0 --> agent does not intersect with other agents"
        E_msg = "Permutaion matrix for swapping agents. Each row has num_agent*num_agents elements that represent the successive rows of E: E[1,:], E[2,:], ... E[num_agents,:]. The data is synchronized with the timestamp"
        stopFlag_msg = "Flags that indicate whether the drone should be moving or not. Each row corresponds to an iteration and consists of num_agents flags each belonging to an agents. The data is synchronized with the timestamp"
        stopFlag_old_msg = "Flags that indicate whether the drone should be moving or not (Before grid-lock resolution). Each row corresponds to an iteration and consists of num_agents flags each belonging to an agents. The data is synchronized with the timestamp"
        colIdx_msg = "Collision indicies for all agents. Each row represnets an agent's collision indicies. Every num_agents rows correspond to an iteration. The latter are synchronized with the timestamp. The format becomes: row1: colIdx(agent1 @ t=1), row2: colIdx(agent2 @ t=1), ... row_num_agents: colIdx(agent_num_agents @ t=1), row_(num_agents+1): colIdx(agent1 @ t=2), row_(num_agents+1): colIdx(agent2 @ t=2), ... row_(2*num_agents): colIdx(agent_num_agents @ t=1) ..."

        saved_data_path = os.path.abspath("distributed_formation_control/saved_data")
        saved_data_path = saved_data_path + "/" + str_time
        if not os.path.exists(saved_data_path):
            os.makedirs(saved_data_path)

        store_data_csv(file_name = saved_data_path + '/time_stamp', data = timestamp, msg = timestamp_msg)
        store_data_csv(file_name = saved_data_path + '/positions', data = q_all, msg = pos_msg)
        store_data_csv(file_name = saved_data_path + '/control_velocity', data = dq_all, msg = ctrl_vel_msg)
        store_data_csv(file_name = saved_data_path + '/control_distance', data = ctrl_all, msg = ctrl_pos_msg)
        store_data_csv(file_name = saved_data_path + '/desired_position', data = q_des_calc, msg = des_pos_msg)
        store_data_csv(file_name = saved_data_path + '/safe_desired_position', data = q_des_safe, msg = safe_des_pos_msg)
        store_data_csv(file_name = saved_data_path + '/safet_net_data', data = safety_net_data_all, msg = safety_net_msg)
        store_data_csv(file_name = saved_data_path + '/permutations', data = E_all, msg = E_msg)
        store_data_csv(file_name = saved_data_path + '/stopFlag', data = stopFlag_all, msg = stopFlag_msg)
        store_data_csv(file_name = saved_data_path + '/stopFlag_old', data = stopFlag_old_all, msg = stopFlag_old_msg)
        store_data_csv(file_name = saved_data_path + '/colIdx', data = colIdx_all, msg = colIdx_msg)

        print('Data saved with extensions:')
        print(str_time)

    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="Run using simulation", action="store_true")
    args = parser.parse_args()
    # try:
    main(args)
    # except:
    #     print('Error occured')
