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
from geometry_msgs.msg import Twist,Pose,PoseStamped,Point,Quaternion,PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction,MoveBaseActionGoal
from people_msgs.msg import People,Person

class target_reporter:

  def __init__(self):
    #publisher
    #self.teleop_pub = rospy.Publisher("/vector/teleop/cmd_vel",Twist)
    #self.move_base_goal =rospy.Publisher("move_base_navi/goal",MoveBaseActionGoal,queue_size=10)
    self.target_position = rospy.Publisher("/tracked_target",People,queue_size=5)

    #frames
    self.target_acquired = False
    self.pub_target = PoseStamped()
    self.target_name = ""
    self.hlpr_found = False
    self.hlpr_position = Point()
    #subscribers
    self.people_sub = rospy.Subscriber("/people_tracker/People",People,self.people_tracker_callback)
    self.amcl_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_pose_callback)
    self.tf = tf.TransformListener();
    

    
  def amcl_pose_callback(self,data):
    self.hlpr_position = data.pose.pose.position
    self.hlpr_found = True

  def point_distance(self,point1,point2)
    dist = math.sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2) + pow(point1.z - point2.z,2))
    return dist

  def velocity2orientation(self,velocity)
    
    
  def people_tracker_callback(self,data):
    if self.tf.frameExists(data.header.frame_id):
      #if follow target is not set, follow nearest Person to base_link
      if self.target_acquired = False
      	if len(data.people) > 0
          distances = []
          for i in range(len(data.people))
            distances.append(0)
            distances[i] = self.point_distance(data.people[i].position,self.hlpr_position)
          mindex = distances.index(min(distances))
          self.target_name = data.people[mindex].name
          self.pub_target.Header = data.Header;
          self.pub_target.pose.position = data.people[mindex].position
          self.pub_target.pose.orientation.x = 0
          self.pub_target.pose.orientation.y = 0
          self.pub_target.pose.orientation.z = 1
          self.pub_target.pose.orientation.w = math.atan(data.people[mindex].velocity.y / data.people[mindex].velocity.x)
          self.target_acquired = True
      #if follow target is set, look for follow target in Person array
      else
        for i in range(len(data.people))
          if data.people[i].name == self.target_name
            self.pub_target.Header = data.Header;
            self.pub_target.pose.position = data.people[i].position
            self.pub_target.pose.orientation.x = 0
            self.pub_target.pose.orientation.y = 0
            self.pub_target.pose.orientation.z = 1
            self.pub_target.pose.orientation.w = math.atan(data.people[mindex].velocity.y / data.people[mindex].velocity.x)

      if target_acquired
        target_position.publish(pub_target)

    else:
      rospy.logwarn("Cannot find the frame %s", data.header.frame_id)


def main(args):
  rospy.init_node('target_reporter', anonymous=True)
  ic = target_reporter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
