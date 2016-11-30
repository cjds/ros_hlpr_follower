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
import numpy as np

class CollisionChecking:

  '''
	This is the information class. It has the base tuple that the other classes use
  '''
  def __init__(self, buffer, map_size):

    self.map_size=map_size
  	self.costmap=np.zeros((map_size,map_size)) #the YML file

  	self.laser_scan_data=0 #laser scan data

  	self.buffer=buffer #this is the buffer around the objects 


  def laser_scan_data(self,data):
  	self.laser_scan_data=data
    kernel_size=self.buffer*2
    center_max=10.0   
    self.costmap=np.zeros((self.map_size,map_size)) #the YML file

    #convert laser scan to data
    number_of_ranges=len(data.ranges)
    for iterator,range_data in enumerate(data.ranges):
      angle=data.angle_min+((data.angle_max - data.angle_min)*iterator/number_of_ranges)
      if range_data>data.range_min and range_data<=data.range_max:
        x= self.map_size*math.cos(angle)*range_data/data.range_max
        y= self.map_size*math.cos(angle)*range_data/data.range_max
        #based on the kernel add the postitions to the costmaps. ,
        for x_pos in range(-kernel_size,kernel_size):
          for y_pos in range(-kernel_size,kernel_size):
            self.costmap[max(0, min(x, self.map_size))][max(0, min(y, self.map_size))]=center_max/(math.abs(x_pos)*math.abs(y_pos))

  def collision_check(self,x,y,theta):
    #check if it laser scan data
    #return the costmap for that footprint
    #todo
  	return self.costmap[x][y]



