#!/usr/bin/env python
'''
We resolve possible collisions or leaving the fliable region by reassigning the goal position of crazyflies.

We take as input parameters for the rectangular prism flyable region (x,y,z max and min), the safe ball around each crazyflie, the desired postion of the crazyflies and the current postisions. Then, we compare the desired positions to make sure that non of them are outside the safet zone or intersect with the safety ball around individual crazyflies. If unsafe desired postion, the current position is issued as desired.

Author:
Sleiman Safaoui
Github:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

July 1, 2018
'''
import numpy as np

class FlyableRegion:
    '''
    coordinates for the rectangular prism flyable region
    '''
    def __init__(self):
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.min_z = 0
        self.max_z = 0
        self.coordinates_set = False # check if coordinates have been set

    def set_area(self, min_x, max_x, min_y, max_y, min_z, max_z):
        '''
        sets the flyable region coordinates
        returns true if successful
        returns false if not successful
        '''
        if ((min_x >= max_x) | (min_y >= max_y) | ( min_z >= max_z)):
            self.min_x = 0
            self.max_x = 0
            self.min_y = 0
            self.max_y = 0
            self.min_z = 0
            self.max_z = 0
            self.coordinates_set = False
            print("Incorrect input. Cannot set coordinates")
            return False

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z
        self.coordinates_set = True
        return True

    def get_area(self):
        '''
        returns the coordinates of the flyable area
        '''
        if self.coordinates_set == True:
            return [min_x, max_x, min_y, max_y, min_z, max_z]
        else:
            return []

class SafetyNet(FlyableRegion):
    '''
    resolves possible collsions/leaving the flyable region
    usage pseudo code:
        SafetyNet.set_params
        Loop:
            get current and desired positions in format: [[x1,y1,z1], [x2,y2,z2] ...]
            SafetyNet.update_pos
            if SafetyNet.find_des_pos_safe
                SafetyNet.get_des_pos_safe
            else # could not find safe desired positions
                Do something else
    '''
    def __init__ (self):
        self.rect = FlyableRegion() # flyable region
        self.ball_radius = 0 # radius of safety ball around crazyflie
        self.initialized = False # check if parameters have been initialized
        self.num_agents = 0 # number of crazyflies
        self.current_pos = [] # current position of crazyflies
        self.des_pos = [] # desired position of the crazyflies
        self.pos_set = False # check if current and desired positions have been set properly
        self.des_pos_safe = [] # safe desired position

    def set_params(self, min_x, max_x, min_y, max_y, min_z, max_z, ball_radius, num_agents):
        '''
        sets the parametes for the safety net
        returns true if successful
        returns false if not successful
        '''
        if ball_radius < 0:
            print('Invalid ball_radius')
            self.initialized = False
            return False
        if num_agents <= 0:
            print('Invalid number of agents')
            self.initialized = False
            return False

        set_area_check = self.rect.set_area(min_x, max_x, min_y, max_y, min_z, max_z)
        if set_area_check == False:
            print('Could not set area')
            self.initialized = False
            return False

        self.num_agents = num_agents
        self.ball_radius = ball_radius
        self.initialized = True
        return True

    def update_pos(self, current_pos, des_pos):
        '''
        sets current and desired positions
        returns true if successful
        returns false if not successful
        '''
        if ((list(current_pos) == []) | (len(list(current_pos)) != self.num_agents)) :
            print('Invalid current positions')
            self.pos_set = False
            return False

        if ((list(des_pos) == []) | (len(list(des_pos)) != self.num_agents)):
            print('Invalid desired positions')
            self.pos_set = False
            return False

        self.current_pos = list(current_pos)
        self.des_pos = list(des_pos)
        self.pos_set = True
        # self.des_pos_safe = []
        return True

    def in_flyable(self, p):
        '''
        checks is a point p is in the flyable area

        input:
                p: list conaining the 3D coordinates [x,y,z]
        output:
                True if the point belongs to the flyable region and False otherwise
        '''
        try:
            x = p[0]
            y = p[1]
            z = p[2]
            # check if the point is outside the flyable area
            is_outside = ( (x>self.rect.max_x) | (x<self.rect.min_x) | (y>self.rect.max_y) | (y<self.rect.min_y) | (z>self.rect.max_z) | (z<self.rect.min_z) )
            if is_outside == True:
                return False # ouside the flyable area
            else:
                return True # inside the flyable area
        except:
            return False

    def find_des_pos_safe(self):
        '''
        finds des_pos_safe based on data in self
        returns True, outside_flyable, intersection if des_pos_safe found
        returns False, [], [] if des_pos_safe not found
        '''
        if self.initialized == False: # No parameters to use
            print('No parameters to use')
            self.des_pos_safe = []
            return False, [], []

        if self.pos_set == False: # Positions not updated'
            print('No positions set')
            self.des_pos_safe = []
            return False, [], []

        len_current_pos = len(self.current_pos)
        len_des_pos = len(self.des_pos)
        if ((self.num_agents != len_current_pos) | (self.num_agents != len_des_pos)): # Size of current and desired positions do not match number of agents
            print('Size of current and desired positions do not match number of agents')
            self.des_pos_safe = []
            return False, [], []

        # loop through all the agents, compare distances to safe distances, and assing values for des_pos_safe
        des_pos_safe_temp = [] # temporary list for the safe desired positions
        intersection = list(np.zeros(self.num_agents)) # list of checks to indicate if agents collide
        outside_flyable = list(np.zeros(self.num_agents)) # list of checks to indicate if an agent is outside the flyable region
        for i in range(self.num_agents):
            # check is des_pos is outside flyable area
            # print("checking point: ", self.des_pos[i])
            if self.in_flyable(self.des_pos[i]): # crazyflie inside the flyable area
                # print('point in flyable region\n checking for possible intersection with neighboring cfs')
                # check for intersections with neighboring cfs
                for j in range(i,self.num_agents):
                    if ((j != i)):
                        dist = l2_dist_in_3d(self.des_pos[i], self.des_pos[j]) # distance between the two desired positions in 3D
                        # print("dist", dist)
                        if (dist - 2*self.ball_radius) < 0: # the two balls around the robots intersect
                            intersection[i] = 1
                            intersection[j] = 1
                if intersection[i] == 1: # intersection exists
                    # print('intersection detected')
                    if self.des_pos_safe == []: # no safe position detected yet
                        des_pos_safe_temp.append(self.current_pos[i]) # use current position for now
                    else: # previous safe position existed
                        des_pos_safe_temp.append(self.des_pos_safe[i]) # use old safe position as safe one
                else:
                    # print('NO intersections detected')
                    des_pos_safe_temp.append(self.des_pos[i]) # no intersections --> assign desired position as safe desired position

            else: # outside the flyable area
                outside_flyable[i] = 1
                # print('point outside flyable region')
                if self.des_pos_safe == []: # no safe position detected yet
                    des_pos_safe_temp.append(self.current_pos[i]) # use current position for now
                else: # previous safe position existed
                    des_pos_safe_temp.append(self.des_pos_safe[i]) # use old safe position as safe one

        self.des_pos_safe = des_pos_safe_temp[:]
        # print('Safe positions found:')
        # print(self.des_pos_safe)
        return True, outside_flyable, intersection

    def get_des_pos_safe(self):
        '''
        returns des_pos_safe
        '''
        return self.des_pos_safe

def get_current_pos_list(cfs):
    '''
    returns a list containing the current position of all crazyflies

    input:
            cfs: allcfs.crazyflies object
    output:
            current_pos: list containing n (num agents) lists of the x, y, and z postion of the crazyflies [[x1, y1, z1], [x2, y2, z2], ... [xn, yn, zn]]

    '''
    current_pos = []

    for cf in cfs:
        pos = cf.position()
        current_pos.append( [ pos[0], pos[1], pos[2] ] )

    return current_pos

def get_current_rot_list(cfs):
    '''
    returns a list containing the current quatornian rotations of all crazyflies

    input:
            cfs: allcfs.crazyflies object
    output:
            rotations: list containing n (num agents) lists of the x, y, z, and w quatornians of the crazyflies [[x1, y1, z1, w1], [x2, y2, z2, w2], ... [xn, yn, zn, wn]]

    '''
    rotations = []

    for cf in cfs:
        rot = cf.rotation()
        rotations.append( [ rot[0], rot[1], rot[2], rot[3] ] )

    return rotations


def l2_dist_in_3d (p1, p2):
    '''
    finds the Euclidean distance between two points in 3d

    input:
            p1, p2: 2 points of the form [x,y,z]
    output:
            float representing the distance between points p1 and p2 in the 3D space
    '''

    p1 = list(p1)
    p2 = list(p2)

    d = 0
    for i in range(3):
        d += (float(p1[i]) - float(p2[i]))**2
    d = d**0.5

    return d
