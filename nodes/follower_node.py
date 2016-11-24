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

class follower_node:

  def __init__(self):
    #publisher
    self.teleop_pub = rospy.Publisher("/vector/teleop/cmd_vel",Twist)

    #frames
    self.radius= 5.0 #radius at which its considered acceptable to move
    #subscribers
    self.image_teleop_sub = rospy.Subscriber("/person_tracker",PoseStamped,self.person_tracker_callback)
    rospy.init_node('follower_node')
    self.tf = tf.TransformListener()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      r.sleep()


  def person_tracker_callback(self,data):
    if self.tf.frameExists(data.header.frame_id):
      #find how far the person is from you. 
      #If he is far enough move in his direction

      pose=self.tf.transformPose("/kinect_rgb_optical_frame",data)
      twist=Twist()
      #I assume that I do not care about the y dimension which is the person's height
      if math.hypot(pose.pose.position.x,pose.pose.position.z) > self.radius:
        #we are not at the goal time to move there
        twist.linear.x=0.1
        twist.linear.y=0.0
        twist.linear.z=0.0
        self.teleop_pub.publish(twist)
      else:
        twist.linear.x=0.0
        twist.linear.y=0.0
        twist.linear.z=0.0
        self.teleop_pub.publish(twist)
    else:
      rospy.logwarn("Cannot find the frame %s", data.header.frame_id)


def main(args):
  ic = follower_node()
  rospy.init_node('follower_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
