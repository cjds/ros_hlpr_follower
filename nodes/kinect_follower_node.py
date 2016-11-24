#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import cv2
import tf 
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class kinect_follower:

  def __init__(self):
  	#objects for python
    self.bridge = CvBridge()


    #publisher
    self.person_tracker_pub = rospy.Publisher("/person_tracker",PoseStamped,queue_size=10)

    #subscribers
    self.image_sub = rospy.Subscriber("image_topic",Image,self.image_received) #
    self.teleop_pub = rospy.Publisher("/vector/teleop/cmd_vel",Twist,queue_size=10)
    rospy.init_node('kinect_follower_node')
    self.tf = tf.TransformListener()
    self.pose= PoseStamped()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
		#probably publish a useful pose here
		self.pose.header.frame_id="/kinect_rgb_optical_frame"
		self.pose.header.stamp = rospy.get_rostime()
		self.pose.pose.position.x=1
		self.pose.pose.position.y=0
		self.pose.pose.position.z=1
		self.person_tracker_pub.publish(self.pose)
		r.sleep()


  def image_received(self,data):
  	#TODO: replace with useful code that does things
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = kinect_follower()
  rospy.init_node('kinect_follower_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
