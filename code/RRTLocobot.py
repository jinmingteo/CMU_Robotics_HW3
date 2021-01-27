import os
import time
import pickle
import argparse
import numpy as np
import matplotlib.pyplot as plt
from os.path import dirname, join, abspath

from pyrobot import Robot
import RobotUtil as rt
import Locobot


def FindNearest(prevPoints,newPoint):
	# You can use this function to find nearest neighbors in configuration space
	D=np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
	return D.argmin()


def main(args):
	# NOTE: Please set a random seed for your random joint generator so we can get the same path as you if we run your code!
	np.random.seed(0)
	deg_to_rad = np.pi/180.

	#Initialize robot object
	mybot=Locobot.Locobot()

	#Create environment obstacles
	pointsObs=[]
	axesObs=[]

	envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,-0.15,0.]),[0.1,0.1,1.05])
	pointsObs.append(envpoints), axesObs.append(envaxes)

	envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.1,0.0,0.675]),[0.45,0.15,0.1])
	pointsObs.append(envpoints), axesObs.append(envaxes)

	envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.275,0.0,0.]),[0.1,1.0,1.25])
	pointsObs.append(envpoints), axesObs.append(envaxes)

	# base and mount
	envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.,0.0,0.05996]),[0.35004,0.3521,0.12276])
	pointsObs.append(envpoints), axesObs.append(envaxes)

	envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.03768,0.0,0.36142]),[0.12001,0.26,0.5])
	pointsObs.append(envpoints), axesObs.append(envaxes)

	# Define initial pose	
	qInit=[-80.*deg_to_rad, 0., 0., 0., 0.] 

	# target box to grasp (You may only need the dimensions and pose, not the points and axes depending on your implementation)
	target_coords, target_orientation = [0.5,0.0,0.05], [0,0.,0.]
	targetpoints, targetaxes = rt.BlockDesc2Points(rt.rpyxyz2H(target_orientation,target_coords),[0.04,0.04,0.1])

	#Generate query for a block object (note random sampling in TGoal)
	QGoal=[]
	num_grasp_points = 5 # You can adjust the number of grasp points you want to sample here

	while len(QGoal)<num_grasp_points:
		# TODO: Sample grasp points here, get their joint configurations, and check if they are valid
		TGoal = np.identity(4)

		# add in target_coords
		for i in range(len(target_coords)):
			TGoal[i][3] = target_coords[i]

		# add randomness in x,y,z 
		TGoal[2][3] += np.random.uniform(-0.05,0.05)
		#TGoal[np.random.randint(3)][3] += np.random.uniform(-0.1, 0.1)
	
		ang, Err = mybot.IterInvKin(qInit, TGoal)
		
		if mybot.RobotInLimits(ang) and np.linalg.norm(Err[0:3])< 0.005 and np.linalg.norm(Err[4:6]) < 0.005:
			if not mybot.DetectCollision(ang, pointsObs, axesObs):
				QGoal.append(np.array(ang))

	#Create RRT graph to find path to a goal configuration
	rrtVertices=[]
	rrtEdges=[]

	# initialize with initial joint configuration and parent
	rrtVertices.append(qInit)
	rrtEdges.append(0)

	# Change these two hyperparameters as needed
	thresh=1.5 
	num_samples = 3000

	FoundSolution=False

	while len(rrtVertices)<num_samples and not FoundSolution:
		# TODO: Implement your RRT planner here	
		# Use your sampler and collision detection from homework 2
		# NOTE: Remember to add a goal bias when you sample
		qRand=mybot.SampleRobotConfig() if np.random.uniform() >= 0.05 else QGoal[np.random.randint(len(QGoal))]
		
		idNear = FindNearest(rrtVertices, qRand)
		qNear = rrtVertices[idNear]

		if np.linalg.norm(np.array(qRand) - np.array(qNear)) > thresh:
			qConnect = np.array(qNear) + thresh*(np.array(qRand)- np.array(qNear))/np.linalg.norm(np.array(qRand)-np.array(qNear))
		else:
			qConnect = qRand
		
		if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
			rrtVertices.append(qConnect)
			rrtEdges.append(idNear)

		
		for qGoal in QGoal:
			idNear = FindNearest(rrtVertices, qGoal)
			if np.linalg.norm(qGoal - rrtVertices[idNear]) < 0.025:
				rrtVertices.append(qGoal)
				rrtEdges.append(idNear)
				FoundSolution = True
				break
		
	if FoundSolution:

		# Extract path - TODO: add your path from your RRT after a solution has been found
		# Last in First Out
		plan=[]
		plan.insert(0, np.array(rrtVertices[-1]))
		edge = rrtEdges[-1]
		while edge != 0:
			idNear = FindNearest(rrtVertices, rrtVertices[edge])
			plan.insert(0, np.array(rrtVertices[idNear]))
			edge = rrtEdges[idNear]
		plan.insert(0, np.array(qInit))

		pointsObs.append(targetpoints), axesObs.append(targetaxes)

		# Path shortening - TODO: implement path shortening in the for loop below
		num_iterations = 1500 # change this hyperparameter as needed
		for i in range(num_iterations):
			anchorA = np.random.randint(len(plan)-2)
			anchorB = np.random.randint(anchorA+1, len(plan)-1)

			shiftA = np.random.uniform(0, 1)
			shiftB = np.random.uniform(0, 1)

			candidateA = (1-shiftA) * plan[anchorA] + shiftA * plan[anchorA + 1]
			candidateB = (1-shiftB) * plan[anchorB] + shiftB * plan[anchorB + 1]

			if not mybot.DetectCollisionEdge(candidateA, candidateB, pointsObs, axesObs):
				while anchorB - anchorA:
					plan.pop(anchorB)
					anchorB=anchorB-1
				plan.insert(anchorA+1, candidateB)
				plan.insert(anchorA+1, candidateA)

		if args.use_pyrobot:
			# Vizualize your plan in PyRobot
			common_config = {}
			common_config["scene_path"] = join(
				dirname(abspath(__file__)), "../scene/locobot_motion_hw3.ttt"
			)
		
			robot = Robot("vrep_locobot", common_config=common_config)

			# Execute plan
			for q in plan:
				robot.arm.set_joint_positions(q)
			
			# grasp block
			robot.gripper.close()

		else:
			# Visualize your Plan in matplotlib
			for q in plan:
				mybot.PlotCollisionBlockPoints(q, pointsObs)
			

	else:
		print("No solution found")

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--use_pyrobot', type=bool, default=False)
	args = parser.parse_args()
	main(args)

