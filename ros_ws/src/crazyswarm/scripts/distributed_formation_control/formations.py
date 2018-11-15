#!/usr/bin/env python2
'''
Loads a particular formation for the formation control strategy.

Author:
Sleiman Safaoui
Github:
The-SS
Email:
snsafaoui@gmail.com

Date:
Sept 8, 2018
'''

import numpy as np

def load_shapes(form_ctrl_shape):
    '''
    Summary of shapes:
    - 4 agents:
        - tetra: tetrahedron
    - 5 agents:
        - sq_base_pyramid: pyramid with square base
    - 6 agents:
        - penta_base_pyramid: pyramid with pentagonal base
    - 8 agents:
        - 8_agent_cube: cube
        - parallelepiped: parallelepiped
        - 8_agent_struct_1: A square with another square inscribed in it (corners at midpoint of edges). The latter is elevated
        - 8_agent_struct_2: Similar to 8_agent_struct_1, but with more elevation for inscribed square
    - 9 agents:
        - 9_agent_struct_1:
    - 12 agents:
        - 12_agent_struct_1:
        - 12_agent_struct_2:
    - 16 agents:
        - 16_agent_struct: concentric circles at three hights: center, 5 agents, 10 agents
    '''
    if form_ctrl_shape == 'tetra':
        des_form_scale = 0.6
        dcoll = 0.9
        rcoll = 0.85
        num_agents = 4
        q_des_list = []
        q_des_list.append([  (8./9.)**0.5,          0.    ,    -1./3. ])
        q_des_list.append([ -(2./9.)**0.5,   -(2./3.)**0.5,    -1./3. ])
        q_des_list.append([ -(2./9.)**0.5,    (2./3.)**0.5,    -1./3. ])
        q_des_list.append([        0.    ,          0.    ,     1.0   ])
        adj_array = np.ones([num_agents, num_agents], dtype=float) - np.eye(num_agents, dtype=float) # full graph
    elif form_ctrl_shape == 'sq_base_pyramid':
        des_form_scale = 1.0
        dcoll = 0.55
        rcoll = 0.45
        num_agents = 5
        q_des_list = []
        q_des_list.append([-0.5,  0.0, 0.0])
        q_des_list.append([-0.5, -1.0, 0.0])
        q_des_list.append([ 0.5, -1.0, 0.0])
        q_des_list.append([+0.5,  0.0, 0.0])
        q_des_list.append([ 0.0, -0.5, 1.0])
        adj_array = np.ones([num_agents, num_agents], dtype=float) - np.eye(num_agents, dtype=float) # full graph
    elif form_ctrl_shape == 'penta_base_pyramid':
        des_form_scale = 0.8
        dcoll = 0.55
        rcoll = 0.45
        num_agents = 6
        q_des_list = []
        q_des_list.append([0.9511, 0.3090, 0.0])
        q_des_list.append([0.5878, -0.8090, 0.0])
        q_des_list.append([-0.5878, -0.8090, 0.0])
        q_des_list.append([0.0, 0.0, 1.0])
        q_des_list.append([-0.9511, 0.3090, 0.0])
        q_des_list.append([-0.0000, 1.0000, 0.0])
        adj_array = np.array([\
        [0,  1,  0,  0,  1,  1],
        [1,  0,  1,  0,  0,  1],
        [0,  1,  0,  1,  0,  1],
        [0,  0,  1,  0,  1,  1],
        [1,  0,  0,  1,  0,  1],
        [1,  1,  1,  1,  1,  0]]) # 6 agent pyramid with pentagon base
    elif form_ctrl_shape == '8_agent_cube':
        des_form_scale = 1.0
        dcoll = 0.8
        rcoll = 0.8
        num_agents = 8
        q_des_list = []
        q_des_list.append([0.0, 0.0, 0.0])
        q_des_list.append([0.0, 1.0, 0.0])
        q_des_list.append([1.0, 1.0, 0.0])
        q_des_list.append([1.0, 0.0, 0.0])
        q_des_list.append([0.0, 0.0, 1.0])
        q_des_list.append([0.0, 1.0, 1.0])
        q_des_list.append([1.0, 1.0, 1.0])
        q_des_list.append([1.0, 0.0, 1.0])
        adj_array = np.array([\
        [ 0  ,   1  ,   1  ,   0  ,   1  ,   0  ,   0  ,   0],\
        [ 1  ,   0  ,   0  ,   1  ,   0  ,   1  ,   0  ,   0],\
        [ 1  ,   0  ,   0  ,   1  ,   0  ,   0  ,   1  ,   0],\
        [ 0  ,   1  ,   1  ,   0  ,   0  ,   0  ,   0  ,   1],\
        [ 1  ,   0  ,   0  ,   0  ,   0  ,   1  ,   1  ,   0],\
        [ 0  ,   1  ,   0  ,   0  ,   1  ,   0  ,   0  ,   1],\
        [ 0  ,   0  ,   1  ,   0  ,   1  ,   0  ,   0  ,   1],\
        [ 0  ,   0  ,   0  ,   1  ,   0  ,   1  ,   1  ,   0]]) # 8 agent Cube or Parallelepiped
    elif form_ctrl_shape == 'parallelepiped':
        des_form_scale = 0.8
        dcoll = 0.43
        rcoll = 0.4
        num_agents = 8
        q_des_list = []
        q_des_list.append([0.0, 0.0, 0.0])
        q_des_list.append([0.0, 1.0, 0.0])
        q_des_list.append([1.0, 1.0, 0.0])
        q_des_list.append([1.0, 0.0, 0.0])
        q_des_list.append([0.5, 0.5, 1.0])
        q_des_list.append([0.5, 1.5, 1.0])
        q_des_list.append([1.5, 1.5, 1.0])
        q_des_list.append([1.5, 0.5, 1.0])
        adj_array = np.array([\
        [ 0  ,   1  ,   1  ,   0  ,   1  ,   0  ,   0  ,   0],\
        [ 1  ,   0  ,   0  ,   1  ,   0  ,   1  ,   0  ,   0],\
        [ 1  ,   0  ,   0  ,   1  ,   0  ,   0  ,   1  ,   0],\
        [ 0  ,   1  ,   1  ,   0  ,   0  ,   0  ,   0  ,   1],\
        [ 1  ,   0  ,   0  ,   0  ,   0  ,   1  ,   1  ,   0],\
        [ 0  ,   1  ,   0  ,   0  ,   1  ,   0  ,   0  ,   1],\
        [ 0  ,   0  ,   1  ,   0  ,   1  ,   0  ,   0  ,   1],\
        [ 0  ,   0  ,   0  ,   1  ,   0  ,   1  ,   1  ,   0]]) # 8 agent Cube or Parallelepiped
    elif form_ctrl_shape == '8_agent_struct_1':
        des_form_scale = 1.5
        dcoll = 0.85
        rcoll = 0.75
        num_agents = 8
        q_des_list = []
        q_des_list.append([0    ,  0   ,   0])
        q_des_list.append([1  ,  0     ,  0])
        q_des_list.append([1  ,   1    ,  0])
        q_des_list.append([0  ,   1    ,   0])
        q_des_list.append([0  ,  0.5   , 0.5])
        q_des_list.append([0.5    ,  0 ,  0.5])
        q_des_list.append([1  , 0.5,  0.5])
        q_des_list.append([0.5    ,  1 ,  0.5])
        adj_array = np.array([\
        [0 ,  1 ,  0 ,  1 ,  1 ,  1 ,  0 ,  0], \
        [1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1 ,  0], \
        [0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1], \
        [1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1], \
        [1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  1], \
        [1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0], \
        [0 ,  1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1], \
        [0 ,  0 ,  1 ,  1 ,  1 ,  0 ,  1 ,  0]]) # 8 agent_structure
    elif form_ctrl_shape == '8_agent_struct_2':
        des_form_scale = 1.0
        dcoll = 0.5
        rcoll = 0.4
        num_agents = 8
        q_des_list = []
        q_des_list.append([0.    ,  0.   ,   0.])
        q_des_list.append([1.  ,  0.     ,  0.])
        q_des_list.append([1.  ,   1.    ,  0.])
        q_des_list.append([0.  ,   1.    ,   0.])
        q_des_list.append([0.  ,  0.5   , 1.0])
        q_des_list.append([0.5    ,  0. ,  1.0])
        q_des_list.append([1.  , 0.5,  1.0])
        q_des_list.append([0.5    ,  1. ,  1.0])
        adj_array = np.array([\
        [0 ,  1 ,  0 ,  1 ,  1 ,  1 ,  0 ,  0], \
        [1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1 ,  0], \
        [0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1], \
        [1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1], \
        [1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  1], \
        [1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0], \
        [0 ,  1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1], \
        [0 ,  0 ,  1 ,  1 ,  1 ,  0 ,  1 ,  0]]) # 8 agent_structure
    elif form_ctrl_shape == '9_agent_struct_1':
        des_form_scale = 1.5
        dcoll = 0.85
        rcoll = 0.85
        num_agents = 9
        q_des_list = []
        q_des_list.append([0    ,  0   ,   0])
        q_des_list.append([1  ,  0     ,  0])
        q_des_list.append([1  ,   1    ,  0])
        q_des_list.append([0  ,   1    ,   0])
        q_des_list.append([0  ,  0.5   , 0.25])
        q_des_list.append([0.5    ,  0 ,  0.25])
        q_des_list.append([1  , 0.5,  0.25])
        q_des_list.append([0.5    ,  1 ,  0.25])
        q_des_list.append([0.5    , 0.5    ,  0.5 ])
        adj_array = np.array([\
        [0 ,  1 ,  0 ,  1 ,  1 ,  1 ,  0 ,  0 ,  1], \
        [1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1 ,  0 ,  0], \
        [0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1 ,  1], \
        [1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0], \
        [1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  1 ,  1], \
        [1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  1], \
        [0 ,  1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  1], \
        [0 ,  0 ,  1 ,  1 ,  1 ,  0 ,  1 ,  0 ,  1], \
        [1 ,  0 ,  1 ,  0 ,  1 ,  1 ,  1 ,  1 ,  0]]) # 9 agent_structure
    elif form_ctrl_shape == '12_agent_struct_1':
        des_form_scale = 1.0
        dcoll = 0.55
        rcoll = 0.45
        num_agents = 12
        q_des_list = []
        q_des_list.append([   -0.7500   ,      0      ,   0])
        q_des_list.append([    0.7500   ,      0      ,   0])
        q_des_list.append([         0   ,-0.7500      ,   0])
        q_des_list.append([         0   , 0.7500      ,   0])
        q_des_list.append([   -0.3000,    0.3000   , 0.3000])
        q_des_list.append([    0.3000,    0.3000   , 0.3000])
        q_des_list.append([    0.3000,   -0.3000   , 0.3000])
        q_des_list.append([   -0.3000,   -0.3000   , 0.3000])
        q_des_list.append([   -0.7500   , 0.7500      ,   0])
        q_des_list.append([    0.7500   , 0.7500      ,   0])
        q_des_list.append([    0.7500   ,-0.7500      ,   0])
        q_des_list.append([   -0.7500   ,-0.7500      ,   0])
        adj_array = np.array([\
        [0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  1],
        [ 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1 ,  0],
        [ 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1],
        [ 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1 ,  0 ,  0],
        [ 0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  1 ,  0 ,  0 ,  0],
        [ 0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  0],
        [ 0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0],
        [ 0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  0 ,  0 ,  1],
        [ 1 ,  0 ,  0 ,  1 ,  1 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1],
        [ 0 ,  1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0],
        [ 0 ,  1 ,  1 ,  0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1],
        [ 1 ,  0  , 1 ,  0   ,0   ,0  , 0  , 1 ,  1  , 0 ,  1  , 0]]) # 12 agent_structure
    elif form_ctrl_shape == '12_agent_struct_2':
        des_form_scale = 1.0
        dcoll = 0.55 # 0.45
        rcoll = 0.45 # 0.4
        num_agents = 12
        q_des_list = []
        q_des_list.append([-0.1500,    0.1500,         0]) #
        q_des_list.append([ 0.1500,    0.1500,         0]) #	9---------------------------10
        q_des_list.append([ 0.1500,   -0.1500,         0]) #	|							 |
        q_des_list.append([-0.1500,   -0.1500,         0]) #	|-----5----------------6-----|
        q_des_list.append([-0.4500,    0.4500,    0.3000]) #	|							 |
        q_des_list.append([ 0.4500,    0.4500,    0.3000]) #	|---------1----------2-------|
        q_des_list.append([ 0.4500,   -0.4500,    0.3000]) #	|---------4----------3-------|
        q_des_list.append([-0.4500,   -0.4500,    0.3000]) #	|							 |
        q_des_list.append([-0.7500,    0.7500,    0.6000]) #	|-----8----------------7-----|
        q_des_list.append([ 0.7500,    0.7500,    0.6000]) #	|						     |
        q_des_list.append([ 0.7500,   -0.7500,    0.6000]) #	12--------------------------11
        q_des_list.append([-0.7500,   -0.7500,    0.6000]) #
        adj_array = np.array([\
        [0,   1,   0,   1,   1,   0,   0,   0,   1,   0,   0,   0],\
        [1,   0,   1,   0,   0,   1,   0,   0,   0,   1,   0,   0],\
        [0,   1,   0,   1,   0,   0,   1,   0,   0,   0,   1,   0],\
        [1,   0,   1,   0,   0,   0,   0,   1,   0,   0,   0,   1],\
        [1,   0,   0,   0,   0,   1,   0,   1,   1,   0,   0,   0],\
        [0,   1,   0,   0,   1,   0,   1,   0,   0,   1,   0,   0],\
        [0,   0,   1,   0,   0,   1,   0,   1,   0,   0,   1,   0],\
        [0,   0,   0,   1,   1,   0,   1,   0,   0,   0,   0,   1],\
        [1,   0,   0,   0,   1,   0,   0,   0,   0,   1,   0,   1],\
        [0,   1,   0,   0,   0,   1,   0,   0,   1,   0,   1,   0],\
        [0,   0,   1,   0,   0,   0,   1,   0,   0,   1,   0,   1],\
        [0,   0,   0,   1,   0,   0,   0,   1,   1,   0,   1,   0]])  #12 agent pyramid formation (no head)
    elif form_ctrl_shape == '16_agent_struct':
        des_form_scale = 1.00
        dcoll = 0.55
        rcoll = 0.45
        num_agents = 16
        q_des_list = []
        q_des_list.append([0.     ,    0.   ,     0.])
        q_des_list.append([0.1545  ,  0.4755  ,  0.5000])
        q_des_list.append([-0.4045  ,  0.2939  ,  0.5000])
        q_des_list.append([-0.4045  , -0.2939  ,  0.5000])
        q_des_list.append([0,.1545  , -0.4755  ,  0.5000])
        q_des_list.append([0,.5000  ,      0  ,  0.5000])
        q_des_list.append([0,.8090  ,  0.5878  ,  1.0000])
        q_des_list.append([0,.3090  ,  0.9511  ,  1.0000])
        q_des_list.append([-0.3090  ,  0.9511  ,  1.0000])
        q_des_list.append([-0.8090  ,  0.5878  ,  1.0000])
        q_des_list.append([-1.0000  ,       0  ,  1.0000])
        q_des_list.append([-0.8090  , -0.5878  ,  1.0000])
        q_des_list.append([-0.3090  , -0.9511  ,  1.0000])
        q_des_list.append([0,.3090  , -0.9511  ,  1.0000])
        q_des_list.append([0,.8090  , -0.5878  ,  1.0000])
        q_des_list.append([1,.0000  ,    0  ,  1.0000])

        adj_array = np.array([\
        [0 , 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1],\
        [1,  0,  1,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1],\
        [1,  1,  0,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0],\
        [1,  0,  1,  0,  1,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0],\
        [1,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0],\
        [1,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0],\
        [1,  1,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1],\
        [1,  0,  1,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0],\
        [1,  0,  1,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  0],\
        [1,  0,  0,  1,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0],\
        [1,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0],\
        [1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0],\
        [1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0],\
        [1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1,  0,  1,  0],\
        [1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0,  1],\
        [1,  1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0]])
        # adj_array = np.array([\
        # [0 , 1,  1,  1,  1,  1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  1],\
        # [1,  0,  1,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1],\
        # [1,  1,  0,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0],\
        # [1,  0,  1,  0,  1,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0],\
        # [1,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0],\
        # [1,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0],\
        # [0,  1,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1],\
        # [0,  0,  1,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0],\
        # [0,  0,  1,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0,  0],\
        # [0,  0,  0,  1,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0,  0],\
        # [1,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0,  0],\
        # [0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0,  0],\
        # [0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0],\
        # [0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1,  0,  1,  0],\
        # [0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0,  1],\
        # [1,  1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0]])
    else:
        print("Invalid formation control shape. ABORTING.")
        des_form_scale = 0.0
        dcoll = 0.0
        rcoll = 0.0
        num_agents = 0
        q_des_list = []
        adj_array = np.array([])

    return des_form_scale, dcoll, rcoll, num_agents, q_des_list, adj_array
