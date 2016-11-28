#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import tf 
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped
from sensor_msgs.msg import Image


class Node:
	def __init__(self,p,q):
		self.p=Information()
  		self.q=Information()
  		self.children=[]

  	def addChild(self,node):
  		self.children.append(node)


class StateTree:

  '''
	This is the information class. It has the base tuple that the other classes use
  '''
  def __init__(self, collision_checker):

  	self.collision_check=collision_checker
 
 	self.bmax=3
  	self.dmax=2

  	self.p=Information()
  	self.q=Information()

  	self.root=Node()

  	self.Q= PriorityQueue()

  def getAvailableActions(q):
  	pass

  def plan(self,q):
  	current_node=[]

  	self.Q.put(q)

  	while True:
  		if self.Q.empty():
  			return self.backtrace(qbest)
  		current_node.append(Q.get())
  		L=[]
  		for action in self.getAvailableActions(q):
  			L.append(action)

		L=(sorted(L, key=lambda x: x.modified, reverse=True))  	
  '''
  	Update this tree 
  '''
  def updateTree(self):
  	pass

  '''
  	Update the values in the tree by recreating the root
  '''
  def resolveTree(self):
  	pass

  def goal(self,p,q):
  	return 0

  def utility(self,node):
  	return 0

  def getNodes(sel.f):
  	nodes=[]
  	for



