#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import tf 
import math
from  Information import *
from CollisionChecking import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped
from sensor_msgs.msg import Image
import numpy as np
import Queue
import time

def get_time():
	return time.time()

class Node:
	def __init__(self,q):
		
  		self.robot=q
  		self.children=[]
  		self.depth=0
  		self.parent=None

  	def addChild(self,node):
  		node.depth=self.depth+1
  		node.parent=self
  		self.children.append(node)

  	def getUtility(self,person_information,collision_checker):
  		beta_value= 0.95^^self.depth 
  		#goal weight
  		w_g=0.6
  		#obstacle weight
  		w_o=0.4
  		#smoothness weight
  		w_v= 0.5
  		return 0

class StateTree:

  '''
	This is the information class. It has the base tuple that the other classes use
  '''
  def __init__(self, collision_checker):

  	self.collision_checker=collision_checker
 
 	self.bmax=3
  	self.dmax=2

  	self.p=Information()
  	self.q=Information()

  	self.root=Node(self.q)

  	self.Q= Queue.PriorityQueue()

  	#velocity control parameters
  	self.max_velocity_x=0.5
  	self.max_velocity_y=0.5
  	self.max_velocity_yaw= 0.5

	self.step_velocity=0.01

	self.max_accel=0.5
	self.max_accel_yaw=1.0
	self.time= get_time()

	self.current_x_velocity=0.0
	self.current_y_velocity=0.0
	self.current_yaw_velocity=0.0

  def getAvailableActions(self,node):
  	q=node.robot
  	dt= get_time() - self.time
  	
  	new_nodes=[]
  	#instead of this based on current velocity figure out which is acceptable
  	for v_x in np.arange(q.v_x-self.max_accel*dt,q.v_x+self.max_accel*dt,self.step_velocity):
  		for v_y in np.arange(q.v_y-self.max_accel*dt,q.v_y+self.max_accel*dt,self.step_velocity):
  			for v_theta in np.arange(q.w-self.max_accel_yaw*dt,q.w+self.max_accel_yaw*dt,self.step_velocity):
  				x=q.x +v_x*dt+ math.cos(q.theta)*v_theta*dt
  				y=q.y +v_y*dt+ math.sin(q.theta)*v_theta*dt
  				theta=q.theta + v_theta*dt
  				if not (self.collision_checker.collision_check(x,y,theta)):
  					n=Node(Information(x,y,theta, v_x,v_y,v_theta))
  					node.addChild(n)
					new_nodes.append(n)
  	#check for collision
  	return new_nodes

  	

  def backtrace(self,node):
  	while node.parent!=None:
  		node=node.parent
  	return node


  def plan(self):
  	#WE SHOULD REdefine the root
  	current_node=self.root
  	steps=[]
  	self.Q.put(self.root)

  	while True:
  		if self.Q.empty():
  			self.time=get_time()
  			return self.backtrace(current_node)
  		steps.append(self.Q.get())
  		L= self.getAvailableActions(steps[-1])
  		L.sort(self.compare)

  		for j in range(self.bmax if len(L)>self.bmax else len(L)):
  			if L[j].depth==self.dmax:
  				if L[j].getUtility(self.p)>current_node.getUtility(self.p):
  					current_node=L[j]
  			else:
  				self.Q.put(L[j])
  	#algorithm won't go here yet: TODO put hard time limits for a decision
  	self.time=get_time()
  	return "FAIL"
  '''
  	Update this tree 
  '''
  def updateTree(self):
  	pass

  def compare(self,a,b):
  	a_utility=a.getUtility(self.p) 
  	b_utility=b.getUtility(self.p)
  	if a_utility>b_utility:
  		return 1
  	elif a_utility==b_utility:
  		return 0
  	else:
  		return -1
  	
  	'''
  		Update the values in the tree by recreating the root
  	'''

  def resolveTree(self):
  	pass

  def goal(self,p,q):
  	return 0


  def getNodes(self):
  	nodes=[]
  	



