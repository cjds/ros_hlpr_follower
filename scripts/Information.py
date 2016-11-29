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

class Information:

  '''
	This is the information class. It has the base tuple that the other classes use
  '''
  def __init__(self,x=0,y=0,theta=0,v_x=0,v_y=0,w=0):

  	self.x=0
  	self.y=0

  	self.theta=0

  	self.v_x=0
	self.v_y=0
  	self.w=0

