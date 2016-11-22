#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import tf 
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped
from sensor_msgs.msg import Image

class follower_node:

  def __init__(self):
    #publisher
    self.teleop_pub = rospy.Publisher("/vector/teleop/cmd_vel",Twist)

    #frames

    #subscribers
    self.image_teleop_sub = rospy.Subscriber("/person_tracker",PoseStamped)
    rospy.init_node('follower_node')
    self.tf = tf.TransformListener()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      r.sleep()

  def keyboard_callback(self,data):
    pass

  def pose_callback(self,data):
    if self.tf.frameExists(self.header.frame_id):
      twist=Twist()
      twist.linear.x=0.1
      twist.linear.y=0.0
      twist.linear.z=0.0
      self.teleop_pub.publish(twist)
    else:
      rospy.logwarn("Cannot find the frame %s", self.header.frame_id)


def main(args):
  ic = follower_node()
  rospy.init_node('follower_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
