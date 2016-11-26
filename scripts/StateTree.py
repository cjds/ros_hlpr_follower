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

  '''
  	Update this tree 
  '''
  def updateTree():
  	pass

  def resolveTree():
  	pass

  def goal(p,q):
  	return 0

  def utility():
  	return 0



