#!/usr/bin/env python
from __future__ import print_function
import roslib
import actionlib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import tf 
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped,Point,Quaternion
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction,MoveBaseActionGoal

class follower_node:

  def __init__(self):
    #publisher
    #self.teleop_pub = rospy.Publisher("/vector/teleop/cmd_vel",Twist)
    #self.move_base_goal =rospy.Publisher("move_base_navi/goal",MoveBaseActionGoal,queue_size=10)

    #frames
    self.radius= 1.0 #radius at which its considered acceptable to move
    #subscribers
    self.image_teleop_sub = rospy.Subscriber("/person_tracker",PoseStamped,self.person_tracker_callback)
    self.image_teleop_pub = rospy.Publisher("/person_tracker",PoseStamped,queue_size=10)
    self.tf = tf.TransformListener()
    size=10
    positions=[]
    self.client = actionlib.SimpleActionClient('move_base_navi', MoveBaseAction)
    self.client.wait_for_server()
    

    for i in range(size):
      positions.append(Point())
      positions[i].x=0.5 + (i*0.05)
      positions[i].y=0.5 + (i*0.05)
      positions[i].z=0.0
    orientation=[]
    for i in range(size):
      orientation.append(Quaternion())
      orientation[i].w=0.5 + (i*0.05)
      orientation[i].z=0.5 + (i*0.05)
      orientation[i].y=0.0
      orientation[i].x=0.0
    r = rospy.Rate(5) # 10hz
    count=0
    while not rospy.is_shutdown():
      new_pose=PoseStamped()
      new_pose.header.frame_id='/map'
      new_pose.header.stamp=rospy.Time.now()
      new_pose.pose.position=positions[count%size]
      new_pose.pose.orientation=orientation[count%size]
      count+=1
      self.image_teleop_pub.publish(new_pose)
      r.sleep()


  def person_tracker_callback(self,data):
    if self.tf.frameExists(data.header.frame_id):
      #find how far the person is from you. 
      #If he is far enough move in his direction

      pose=self.tf.transformPose("/map",data)
      twist=Twist()
      #I assume that I do not care about the y dimension which is the person's height
      if math.hypot(pose.pose.position.x,pose.pose.position.y) > self.radius:
        #we are not at the goal time to move there
        move_base_navi=MoveBaseGoal()
        pose.pose.position.x-=0.05
        pose.pose.position.y-=0.05      
        move_base_navi.target_pose.header.frame_id='/map'
        move_base_navi.target_pose.header.stamp=rospy.Time.now()

        move_base_navi.target_pose.pose=pose.pose
        #move_base_navi.goal_id=1
        self.client.send_goal(move_base_navi)

        rospy.logwarn(str(self.client.get_result()))

    else:
      rospy.logwarn("Cannot find the frame %s", data.header.frame_id)


def main(args):
  rospy.init_node('follower_node', anonymous=True)
  ic = follower_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
