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

class CollisionChecking:

  '''
	This is the information class. It has the base tuple that the other classes use
  '''
  def __init__(self, buffer):

  	self.costmap=0 #the YML file

  	self.laser_scan_data=0 #laser scan data

  	self.buffer=buffer #this is the buffer around the objects 


  def laser_scan_data(self,data):
  	self.laser_scan_data=data

  def collision_check():
  	return False



