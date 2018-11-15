#!/usr/bin/env python
'''
Distributed collision avoidance by rotating the control vectors.

Contains functions for resolving collisions in a plane and in the 3D scenario.

Kaveh Fathian:
Email: kaveh.fathian@gmail.com
Github: KavehFathian

Date:
Aug 31, 2018

(C) Kaveh Fathian, 2018.  Email: kaveh.fathian@gmail.com
'''


from __future__ import print_function
import numpy as np
import math
import copy
import time

################################################################################
# Helper functions

def wrapTo180(ang): # Map an angle to the interval [-180, 180]
	if ang == 180:
		return 180.
	else:
		return float( (ang + 180) % 360 - 180 )

def CheckFeasibility(hed, posi, posj, rcoll): # checks if moving along a particular direction would cause a collision
	'''
	inputs:
		- hed: desired motion direction
		- posi: current position of the agent
		- posj: position of adjacent agent
		- rcoll: collision avoidance circle radius
	outputs:
		- colChk: boolian. True when direction of motion intersects the collision region
		- colDist: minimum distance to adjacent agent along the direction of motion
	'''
	hed = np.array(hed, dtype=float)
	posi = np.array(posi, dtype=float)
	posj = np.array(posj, dtype=float)

	nrmHed = np.linalg.norm(hed) # find the norm of the heading
	if nrmHed == 0: # no heading --> no collision
		colChk = False
		colDist = np.inf
		return colChk, colDist

	hed = hed/nrmHed # normalize the heading direction

	vec = posj - posi # vector from agent to adjacent agent

	dotVal = np.dot(hed, vec) # dot product of hed and vec

	if dotVal <=0: # hedaing is not towards the adjacent agent --> no collision
		colChk = False
		colDist = np.inf
		return colChk, colDist
	else: # heading direction is towards the agent
		# find the minimum distance to the adjacent agent along the desired line of motion
		colDist = np.linalg.norm(dotVal*hed - vec)

		# check collision criteria
		if colDist <= rcoll: # if the line intersects the safety sphere
			colChk = True # collision will occure
		else:
			colChk = False
		return colChk, colDist

def CheckFeasibility_2D(hed, posi, posj, rcoll): # checks if moving along a particular direction would cause a collision (projects everything to a plane)
	'''
	inputs:
		- hed: desired motion direction
		- posi: current position of the agent
		- posj: position of adjacent agent
		- rcoll: collision avoidance circle radius
	outputs:
		- colChk: boolian. True when direction of motion intersects the collision region
		- colDist: minimum distance to adjacent agent along the direction of motion
	'''


	hed = np.array(hed, dtype=float)
	posi = np.array(posi, dtype=float)
	posj = np.array(posj, dtype=float)


	# project things into a plane
	posi[2] = 0
	posj[2] = 0
	hed[2] = 0

	nrmHed = np.linalg.norm(hed) # find the norm of the heading
	if nrmHed == 0: # no heading --> no collision
		colChk = False
		colDist = np.inf
		return colChk, colDist

	hed = np.array(hed)/nrmHed # normalize the heading direction

	vec = np.array(posj) - np.array(posi) # vector from agent to adjacent agent

	dotVal = np.dot(hed, vec) # dot product of hed and vec

	if dotVal <=0: # hedaing is not towards the adjacent agent --> no collision
		colChk = False
		colDist = np.inf
		return colChk, colDist
	else: # heading direction is towards the agent
		# find the minimum distance to the adjacent agent along the desired line of motion
		colDist = np.linalg.norm(dotVal*hed - vec)

		# check collision criteria
		if colDist <= rcoll: # if the line intersects the safety sphere
			colChk = True # collision will occure
		else:
			colChk = False
		return colChk, colDist

def SphereSample(n):
	'''
	Distributes n points "equally" about a unit sphere. n The number of points to distribute x,y,z Each is 1 x n vector r The smallest linear distance between two neighboring  points. If the function is run several times for the  same n, r should not change by more than the convergence  criteria, which is +-0.01 on a unit sphere.

	Distributes n points about a unit sphere so that the straight line distance between neighboring points is roughly the same. The actual criteria for stopping is slightly different. The difference between a given point and every other point is stored in a matrix. The iterations stop once the maximum difference between any element in successive distance matrices is less than 0.01. An absolute  criteria was chosen due to self-distances being 0, and programming  around this for relative convergence seemed too much work for too  little reward. The algorithm first generates n random points. Then a repulsive  force vector, based on 1/r^2, is calculated for each point due to  all of the other points. The resultant force vector for each point is normalized, and then each point is displaced a distance s = 1  in the direction of the force. Any value of s between 0.0 and 1. 0 seems to work with most values between 0.2 and 1 taking an average  of 20 to 30 iterations to converge. If s is too high, too much  "energy" is being added to the system and it won't converge. If s is too low, the points won't be evenly distributed even though the convergence criteria is met. Generally speaking, the larger n is the larger s can be without causing instabilities. After  displacement, the point is projected back down onto the unit sphere.  When the system nears convergence, the displacement vector for a  given point is nearly in the same direction as the radius vector for  that point due to the points being equally distributed.

	Inputs:
		- n: number of samples required or number of heads
	Outputs:
		- points:
		- avgr:
	'''
	# Since rand produces number from 0 to 1, subtract off -0.5 so that the points are centered about (0,0,0).
	x = np.random.rand(n) - 0.5
	y = np.random.rand(n) - 0.5
	z = np.random.rand(n) - 0.5

	# Make the matrix R matrices for comparison #TODO: what does this mean
	rm_new = np.ones([n,n])
	rm_old = np.zeros([n,n])

	# Scale the coordinates so that their distance from the origin is 1
	r = np.sqrt(x**2. + y**2. + z**2.)
	x = x/r
	y = y/r
	z = z/r

	not_done = True # boolian to check if done or not

	s = 1.

	while not_done:
		for i in range(n):
			# calculate the i,j,k vecotrs for the direction of the repulsive forces
			ii = x[i] - x
			jj = y[i] - y
			kk = z[i] - z

			rm_new[i,:] = np.sqrt(ii**2. + jj**2. + kk**2.)

			ii = ii/rm_new[i,:]
			jj = jj/rm_new[i,:]
			kk = kk/rm_new[i,:]

			# take care of the self terms
			ii[i] = 0.
			jj[i] = 0.
			kk[i] = 0.

			# Use a 1/r^2 repulsive force, but add 0.01 to the denominator to avoid a 0 * Inf below. The self term automatically disappears since the ii,jj,kk vectors were set to zero for self terms.
			f = 1./(0.01 + rm_new[i,:]**2)

			# Sum the forces.
			fi = np.sum(f*ii)
			fj = np.sum(f*jj)
			fk = np.sum(f*kk)

			# Find magnitude
			fn = np.sqrt(fi**2 + fj**2 + fk**2)

			# Find the unit direction of repulsion.
			fi = fi/fn
			fj = fj/fn
			fk = fk/fn

			# Step a distance s in the direciton of repulsion
			x[i] = x[i] + s*fi
			y[i] = y[i] + s*fj
			z[i] = z[i] + s*fk

			# Scale the coordinates back down to the unit sphere.
			r = np.sqrt(x[i]**2. + y[i]**2. + z[i]**2.)

			x[i] = x[i]/r
			y[i] = y[i]/r
			z[i] = z[i]/r

		# Check convergence
		diff = abs(rm_new - rm_old)

		not_done = np.any(diff > 0.01)

		rm_old = copy.copy(rm_new)

	# Find the smallest distance between neighboring points. To do this exclude the self terms which are 0.
	avgr = []

	points = np.array([list(x), list(y), list(z)])

	return points, avgr

def SphereSample_2D(n):
	'''
	Distributes n points "equally" about a unit sphere. n The number of points to distribute x,y,z Each is 1 x n vector r The smallest linear distance between two neighboring  points. If the function is run several times for the  same n, r should not change by more than the convergence  criteria, which is +-0.01 on a unit sphere.

	Distributes n points about a unit sphere so that the straight line distance between neighboring points is roughly the same. The actual criteria for stopping is slightly different. The difference between a given point and every other point is stored in a matrix. The iterations stop once the maximum difference between any element in successive distance matrices is less than 0.01. An absolute  criteria was chosen due to self-distances being 0, and programming  around this for relative convergence seemed too much work for too  little reward. The algorithm first generates n random points. Then a repulsive  force vector, based on 1/r^2, is calculated for each point due to  all of the other points. The resultant force vector for each point is normalized, and then each point is displaced a distance s = 1  in the direction of the force. Any value of s between 0.0 and 1. 0 seems to work with most values between 0.2 and 1 taking an average  of 20 to 30 iterations to converge. If s is too high, too much  "energy" is being added to the system and it won't converge. If s is too low, the points won't be evenly distributed even though the convergence criteria is met. Generally speaking, the larger n is the larger s can be without causing instabilities. After  displacement, the point is projected back down onto the unit sphere.  When the system nears convergence, the displacement vector for a  given point is nearly in the same direction as the radius vector for  that point due to the points being equally distributed.

	Inputs:
		- n: number of samples required or number of heads
	Outputs:
		- points:
		- avgr:
	'''
	# Since rand produces number from 0 to 1, subtract off -0.5 so that the points are centered about (0,0,0).

	x = np.zeros(n)
	y = np.zeros(n)
	z = np.zeros(n)

	ang = (2*np.pi) / n
	for i in range(n):
		x[i] = np.cos(ang*i)
		y[i] = np.sin(ang*i)

	points = np.array([list(x), list(y), list(z)])
	avgr = []

	return points, avgr

################################################################################
# Resolving collisions in a plane (projects all agents onto a plane and resolves collsions in 2D)
def ColAvoid_Ver2_2(ctrl, qm, n, dcoll, rcoll):
	'''
	ctrl: 2D control signals (2 x num_robots)
	qm: 2D positions of the Robots (2 x num_robots)
	n: number of robots
	dcoll: radius of ball around agent to look for Neighbors
	rcoll: radius of ball around neighbors to avoid
	'''
	inc = 5 # Angle increments

	#  Inter-agent distance matrix
	Dc = np.zeros([n, n], dtype=float)
	for i in range(n):
		 for j in range(n):
			 Dc[i,j] = np.linalg.norm(np.array(qm[:,i]) - np.array(qm[:,j]))
	print('Collision Avoidance interagent distances: ', Dc)


	#  Collision avoidance

	# Index of neighbors inside collision circle
	colIdx = Dc < dcoll
	for i in range(n):
		colIdx[i,i] = False

	# Stop flag to avoid collision
	stopFlag = np.zeros([n,1]);

	for i in range(n): # Agent

		coneAng = [[],[]]; # Angle of cone sides are stored in columns of 'coneAng'

		# Find cone angles
		for k in range(n): # Neighbors
			if colIdx[i,k]: # If collision avoidance is needed
				dnb = Dc[i,k]						# Distance to neighbor
				vec = qm[:,k] - qm[:,i]			  # Vector from agent to its neighbor
				tht = math.degrees(math.atan2(vec[1], vec[0]))		 # Angle of connecting vector

				# 'alp' is the vertex half-angle of the collision cone
				if dnb <= dcoll:
					alp = 90
				else:
					alp = abs( math.degrees( math.asin(rcoll/dnb) ) )

				# Angle of cone sides
				thtm = tht - alp
				thtp = tht + alp

				# Bring all angles to the range [-180, 180] degrees
				if thtm < -180:
					coneAng[0].extend( [wrapTo180(thtm)] )
					coneAng[1].extend( [180.] )
					coneAng[0].extend( [-180.] )
					coneAng[1].extend( [thtp] )
				elif thtp > 180:
					coneAng[0].extend( [-180.] )
					coneAng[1].extend( [wrapTo180(thtp)] )
					coneAng[0].extend( [thtm] )
					coneAng[1].extend( [180.] )
				else:
					coneAng[0].extend( [thtm] )
					coneAng[1].extend( [thtp] )

		if np.any(colIdx[i,]): # If collision avoidance is needed
			# Control vector angle in world coordinate frame
			thtC = math.degrees( math.atan2(ctrl[1,i], ctrl[0,i]) )

			# If control vector is inside a cone change its direction
			if np.any( np.logical_and( thtC >= np.array(coneAng[0]), thtC <= np.array(coneAng[1]) ) ):

				angs = range(-180,18+inc,inc)	# Possible motion directions to test
				angsNum = np.size(angs)
				angsIdx = np.ones(angsNum, dtype=bool)

				# Determine which angles are inside the collision cones
				for k in range(angsNum):
					r = angs[k]
					if np.any( np.logical_and( r >= np.array(coneAng[0]), r <= np.array(coneAng[1]) ) ):
						angsIdx[k] = False

				angsFeas = []
				for k in range(angsNum):
					if angsIdx[k]:
						angsFeas.extend( [angs[k]] ) # Feasible directions to take

				# If there is no feasible angle stop
				if angsFeas == []:
					stopFlag[i] = True
				else:
					# Find closest non-colliding control direction
					thtDiff = []
					for k in range(len(angsFeas)):
						thtDiff.extend( [abs( wrapTo180(thtC - angsFeas[k]) )] )

					minIdx = thtDiff.index( min(thtDiff) )
					thtCnew = angsFeas[minIdx];

					# Check if the feasible control direction is within +-90 degrees, otherwise stop
					if abs( wrapTo180(thtCnew - thtC) ) >= 90:
						stopFlag[i] = True

				# Modified control vector
				if stopFlag[i]:
					ctrl[:,i] = [0,0]
				else:
					ctrl[:,i] = np.linalg.norm(ctrl[:,i]) * \
					np.array( [math.cos(math.radians(thtCnew)), math.sin(math.radians(thtCnew))] )


	return ctrl, stopFlag


################################################################################
# Resolving collisions in 3D
class ColAvoid3D:
	'''
	3D collision avoidance
	'''
	def __init__(self):
		self.hedSamples = 50
		self.heds = []
		self.n = 0 # number of robots/agents
		self.dcoll = 0. # radius of ball around agent to look for Neighbors
		self.rcoll = 0. # radius of ball around neighbors to avoid

	def update_static_params(self, n, dcoll, rcoll): # update static params

		if n <= 0:
			print('Invalid number of agents')
		else:
			self.n = n

		if dcoll <= 0:
			print('Invalid dcoll')
		else:
			self.dcoll = dcoll

		if rcoll <= 0:
			print('Invalid rcoll')
		else:
			self.rcoll = rcoll

		if ((n <= 0) | (dcoll <= 0) | (rcoll <= 0)):
			print('Not all parameters were updated')
			return False

		return True

	def update_sphere_samples(self): # update hed
		self.heds,_ = SphereSample(self.hedSamples)

	def update_sphere_samples_2D(self): # update hed for the 2D case
		self.heds,_ = SphereSample_2D(self.hedSamples)

	def InterAgentDist(self, pos): # calculates inter-agent distances
		n = self.n
		Dc = np.zeros([n,n], dtype=float)
		for i in range(n):
			for j in range(i+1, n):
				Dc[i,j] = np.linalg.norm(np.array(pos)[:,i] - np.array(pos)[:,j])
		Dc = Dc + np.transpose(Dc)
		return Dc

	def InterAgentDist_2D(self, pos): # calculates inter-agent distances after projecting 3D points onto a plane
		n = self.n
		Dc = np.zeros([n,n], dtype=float)
		for i in range(n):
			for j in range(i+1, n):
				Dc[i,j] = np.linalg.norm(np.array(pos)[0:2,i] - np.array(pos)[0:2,j])
		Dc = Dc + np.transpose(Dc)
		return Dc

	def ColAvoid3D_Ver_1_2(self, ctrl, pos): # apply collision avoidance
		'''
		Applies collision avoidance in 3D
		Inputs:
			- ctrl: control signals in 3D (3 x num_robots)
			- pos: 3D position of the robots (3 x num_robots)
		Outputs:
			- ctrl: modified control signal (3 x num_robots)
			- stopFlag: flag to indicate if an agent should stop due to not having a feasible direction of motion
			- colIdx: Index of adjacent agent with which collision occures if no actions are taken
		'''
		# Static parameters check
		n = self.n
		dcoll = self.dcoll
		rcoll = self.rcoll
		hedSamples = self.hedSamples
		if ((n ==0) | (dcoll == 0) | (rcoll == 0)):
			print('Please update static params. ABORTING')
			return [],[],[]

		# Check if the sphere samples have been generated (it determines the feasible motion directions to check)
		if self.heds == []:
			self.update_sphere_samples()
		heds = self.heds

		# change ctrl and pos to arrays (in case they were lists)
		ctrl = np.array(ctrl, dtype=float)
		pos = np.array(pos, dtype=float)

		# Find inter-agent distances
		try:
			Dc = self.InterAgentDist(pos)
		except:
			print('Incorrect use of InterAgentDist. ABORTING')
			return [],[],[]

		# Collision avoidance
		# Check which agents are within the collsion avoidance activation region
		adjIdx = Dc < dcoll
		for i in range(n):
			adjIdx[i,i] = False

		stopFlag = np.array(np.zeros(n), dtype=bool) # Flag to stop agents that do not have a feasible direction of motion
		colIdx = [] # list containing index of adjacent agent with which collision occurs
		for i in range(n):
			colIdx.append([])
		colFlag = np.array(np.zeros(n), dtype=bool) # Flag to see if collision avoidance is required

		for i in range(n):
			posi = copy.copy(pos[:,i]) # position of agent i
			ctr = copy.copy(ctrl[:,i]) # control direction of agent i

			if np.any(adjIdx[i,:]): # if collision avoidance should be considered
				#Find out if the desired direction of motion causes collisions
				indices = [idx for idx, x in enumerate(adjIdx[i,:]) if x == True] # find indices of true elements in adjIdx[i,:]
				for j in indices: # llop through all indices
					posj = copy.copy(pos[:,j]) # position of adjacent agent
					colChk, colDist = CheckFeasibility(ctr, posi, posj, rcoll) # check if collision occurs
					if colChk:
						colIdx[i].extend([j]) # add index of agent with which the collision is most likely to occures
						colFlag[i] = True
				# If collision avoidance is needed, find new direction of motion
				if colFlag[i]:
					stopFlag[i] = True # set stopFlag tp true

					# Find feasible direction of motion
					dotVal0 = 0.
					for k in range(hedSamples):
						hed = copy.copy(heds[:,k]) # a candidate for direction of motion
						# print('hed idx, ',k)
						# print('hed',hed)
						# print('ctr',ctr)
						dotVal = np.dot(np.array(hed, dtype=float), np.array(ctr, dtype=float))
						# print('dotVal', dotVal)
						if dotVal > 0:  # uf the direction is within +-90 degree roation of desired direction
							for j in indices: # Index of adjacent agents
								# print('j',j)
								posj = copy.copy(pos[:,j]) # position of adjacent agent that can cause collision
								colChk, colDist = CheckFeasibility(hed, posi, posj, rcoll) # check if collision occurs
								if colChk:
									break # if a collision is detected exit the loop

							if ((not colChk) & (dotVal > dotVal0)): # if collision does not occure
							 	bestHed = copy.copy(hed) # pick current heading as the best candiate
								dotVal0 = dotVal # update dot product
								stopFlag[i] = False

					if not stopFlag[i]: # if a feasible direction of motion exists
						ctrl[:,i] = bestHed * np.linalg.norm(ctr) # update the control direction to the new heading
		return ctrl, stopFlag, colIdx

	def ColAvoid2D_Ver_1_2(self, ctrl, pos): # apply collision avoidance in 2D
		'''
		Applies collision avoidance in 2D
		Inputs:
			- ctrl: control signals in 3D (3 x num_robots)
			- pos: 3D position of the robots (3 x num_robots)
		Outputs:
			- ctrl: modified control signal (3 x num_robots)
			- stopFlag: flag to indicate if an agent should stop due to not having a feasible direction of motion
			- colIdx: Index of adjacent agent with which collision occures if no actions are taken
		'''
		# Static parameters check
		n = self.n
		dcoll = self.dcoll
		rcoll = self.rcoll
		hedSamples = self.hedSamples
		if ((n ==0) | (dcoll == 0) | (rcoll == 0)):
			print('Please update static params. ABORTING')
			return [],[],[]

		# Check if the sphere samples have been generated (it determines the feasible motion directions to check)
		if self.heds == []:
			self.update_sphere_samples_2D()
		heds = self.heds

		# change ctrl and pos to arrays (in case they were lists)
		ctrl = np.array(ctrl, dtype=float)
		pos = np.array(pos, dtype=float)

		# Find inter-agent distances
		try:
			Dc = self.InterAgentDist_2D(pos)
		except:
			print('Incorrect use of InterAgentDist. ABORTING')
			return [],[],[]

		# Collision avoidance
		# Check which agents are within the collsion avoidance activation region
		adjIdx = Dc < dcoll
		for i in range(n):
			adjIdx[i,i] = False

		stopFlag = np.array(np.zeros(n), dtype=bool) # Flag to stop agents that do not have a feasible direction of motion
		colIdx = [] # list containing index of adjacent agent with which collision occurs
		for i in range(n):
			colIdx.append([])
		colFlag = np.array(np.zeros(n), dtype=bool) # Flag to see if collision avoidance is required

		for i in range(n):
			posi = copy.copy(pos[:,i]) # position of agent i
			ctr = copy.copy(ctrl[:,i]) # control direction of agent i

			if np.any(adjIdx[i,:]): # if collision avoidance should be considered
				#Find out if the desired direction of motion causes collisions
				indices = [idx for idx, x in enumerate(adjIdx[i,:]) if x == True] # find indices of true elements in adjIdx[i,:]
				for j in indices: # llop through all indices
					posj = copy.copy(pos[:,j]) # position of adjacent agent
					colChk, colDist = CheckFeasibility_2D(ctr, posi, posj, rcoll) # check if collision occurs
					if colChk:
						colIdx[i].extend([j]) # add index of agent with which the collision is most likely to occures
						colFlag[i] = True
				# If collision avoidance is needed, find new direction of motion
				if colFlag[i]:
					stopFlag[i] = True # set stopFlag tp true

					# Find feasible direction of motion
					dotVal0 = 0.
					for k in range(hedSamples):
						hed = copy.copy(heds[:,k]) # a candidate for direction of motion
						# print('hed idx, ',k)
						# print('hed',hed)
						# print('ctr',ctr)
						dotVal = np.dot(np.array(hed, dtype=float), np.array(ctr, dtype=float))
						# print('dotVal', dotVal)
						if dotVal > 0:  # uf the direction is within +-90 degree roation of desired direction
							for j in indices: # Index of adjacent agents
								# print('j',j)
								posj = copy.copy(pos[:,j]) # position of adjacent agent that can cause collision
								colChk, colDist = CheckFeasibility_2D(hed, posi, posj, rcoll) # check if collision occurs
								if colChk:
									break # if a collision is detected exit the loop

							if ((not colChk) & (dotVal > dotVal0)): # if collision does not occure
							 	bestHed = copy.copy(hed) # pick current heading as the best candiate
								dotVal0 = dotVal # update dot product
								stopFlag[i] = False

					if not stopFlag[i]: # if a feasible direction of motion exists
						ctrl[:,i] = bestHed * np.linalg.norm(ctr[0:2]) # update the control direction to the new heading
						ctrl[2,i] = ctr[2]
		return ctrl, stopFlag, colIdx


### testing

def main_2D():

	n = 3 # Number of agents
	dcoll = 2 # Collision avaoidance distance
	rcoll = 1 # Collision avaoidance circle radius

	qm = np.array([[1, 2, 3,], [4, 5, 6]])
	ctrl = np.array([[1, 2, 3,], [4, 5, 6]])

	ctrl, stopFlag = ColAvoid_Ver2_2(ctrl, qm, n, dcoll, rcoll)

	print(ctrl)
	print(stopFlag)

def main_3D():
	ctrl = [[0,0,0,1,2], [1,2,-1,0,3], [1,2,1,2,-1]]
	pos = [[100,100,100,110,200],[100,100,100.5,0,20],[19,18,19,250,-120]]
	n = 5
	dcoll = 2
	rcoll = 1
	print(np.array(ctrl))
	print(np.array(pos))
	ca = ColAvoid3D()
	ca.update_static_params(n, dcoll, rcoll)
	Dc = ca.InterAgentDist(np.array(pos))
	ctrl2, stopFlag, colIdx = ca.ColAvoid3D_Ver_1_2(ctrl, pos)
	print(ca.heds)
	print(ctrl2)
	print(stopFlag)
	print(colIdx)

	np.savetxt("pt_cld.csv", ca.heds, delimiter=",")












if __name__ == "__main__":
	# try:
	main_3D()
	# except:
	# 	print("error")
