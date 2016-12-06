#!/usr/bin/env python
from __future__ import print_function
import roslib
import actionlib
roslib.load_manifest('ros_follower_node')
import sys
import rospy
import tf
import random
import copy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose,PoseStamped,Point,Quaternion
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction,MoveBaseActionGoal

class follower_node:

  def __init__(self):
    #publisher

    self.max_count=0
    self.total_count=0

    people_positions=[]
    # people_positions.append([8.339, 4.507, 0.000, 0.000, 0.000, -0.766, 0.643])
    # people_positions.append([7.224, 1.832, 0.000, 0.000, 0.000, -0.850, 0.527])
    # people_positions.append([6.694, 0.998, 0.000,0.000, 0.000, 0.887, -0.462])
    # people_positions.append([6.765, 1.050, 0.000,0.000, 0.000, -0.825, 0.565])
    # people_positions.append([5.716, -1.611, 0.000,0.000, 0.000, -0.516, 0.857])
    # people_positions.append([4.744, -2.189, 0.000,0.000, 0.000, 0.999, 0.049])
    # people_positions.append([2.052, -0.760, 0.000,0.000, 0.000, 0.940, 0.341])
    # people_positions.append([0.454, 0.798, 0.000,0.000, 0.000, 0.930, 0.368])
    # people_positions.append([-1.421, 2.756, 0.000,0.000, 0.000, 0.834, 0.552])
    # people_positions.append([-3.082, 4.578, 0.000,0.000, 0.000, 0.958, 0.288])
    # people_positions.append([-3.853, 7.971, 0.000,0.000, 0.000, 0.137, 0.991])
    # people_positions.append([-2.178, 6.782, 0.000,0.000, 0.000, 0.108, 0.994])

    people_positions.append([2.614, -2.580, 0.000, 0.000, 0.000, -0.703, 0.711])
    people_positions.append([2.423, -5.094, 0.000,0.000, 0.000, 0.961, -0.278])
    people_positions.append([2.365, -6.400, 0.000,0.000, 0.000, -0.799, 0.602])
    people_positions.append([1.848, -7.329, 0.000,0.000, 0.000, 0.964, -0.264])
    people_positions.append([0.689, -7.275, 0.000,0.000, 0.000, 0.885, 0.465])
    people_positions.append([0.685, -6.313, 0.000, 0.000, 0.000, 0.566, 0.824]) 
    people_positions.append([2.153, -5.434, 0.000,0.000, 0.000, 0.364, 0.931])
    people_positions.append([3.162, -4.180, 0.000,0.000, 0.000, 0.364, 0.931]) 
    people_positions.append([3.162, -4.180, 0.000, 0.000, 0.000, 0.502, 0.865])
    people_positions.append([4.689, -2.513, 0.00, 0.000, 0.000, 0.445, 0.896])
    people_positions.append([4.775, -0.466, 0.000,0.000, 0.000, 0.772, 0.636])
    people_positions.append([4.775, -0.466, 0.000,0.000, 0.000, 0.772, 0.636])
    people_positions.append([3.180, 0.721, 0.000,0.000, 0.000, 0.926, 0.377])
    people_positions.append([1.886, 1.627, 0.00,0.000, 0.000, 0.984, 0.176])
    people_positions.append([-0.478, 1.025, 0.000,0.000, 0.000, 0.991, -0.131])
    people_positions.append([0.446, -0.392, 0.000,0.000, 0.000, -0.489, 0.872])

    velocity=80.0
    error_rate=0.05

    size=velocity*(len(people_positions)-1)
    #frames
    self.radius= 1.5 #radius at which its considered acceptable to move
    #subscribers
    self.image_teleop_sub = rospy.Subscriber("/person_tracker",PoseStamped,self.person_tracker_callback)
    self.image_teleop_pub = rospy.Publisher("/person_tracker",PoseStamped,queue_size=10)
    self.visualization_marker_pub = rospy.Publisher("/person_display",Marker,queue_size=10)

    self.tf = tf.TransformListener()
    positions=[]
    self.client = actionlib.SimpleActionClient('move_base_navi', MoveBaseAction)
    self.client.wait_for_server()
    
    #positions and orientations representing poses of the human
    for i in range(len(people_positions)-1):
      for  j in range(int(velocity)):
        c=int(i*velocity + j)
        positions.append(Point())
        positions[c].x=people_positions[i][0]+((people_positions[i+1][0]-people_positions[i][0]) * j / velocity)
        positions[c].y=people_positions[i][1]+((people_positions[i+1][1]-people_positions[i][1]) * j / velocity)
        positions[c].z=people_positions[i][2]+((people_positions[i+1][2]-people_positions[i][2]) * j /  velocity)
    orientation=[]
    for i in range(len(people_positions)-1):
      for  j in range(int(velocity)):
        c=int(i*velocity + j)
        orientation.append(Quaternion())
        orientation[c].x=people_positions[i][3]+((people_positions[i+1][3]-people_positions[i][3]) * j / velocity)
        orientation[c].y=people_positions[i][4]+((people_positions[i+1][4]-people_positions[i][4]) * j / velocity)
        orientation[c].z=people_positions[i][5]+((people_positions[i+1][5]-people_positions[i][5]) * j / velocity)
        orientation[c].w=people_positions[i][6]+((people_positions[i+1][6]-people_positions[i][6]) * j / velocity)


    r = rospy.Rate(10) # 10hz
    count=0
    while not rospy.is_shutdown():
      new_pose=PoseStamped()
      new_pose.header.frame_id='/map'
      new_pose.header.stamp=rospy.Time.now()
      if count < size:
        new_pose.pose.position=positions[int(count%size)]
        new_pose.pose.orientation=orientation[int(count%size)]
        count+=1
        skip_data=False
        if random.random() > error_rate:
          self.image_teleop_pub.publish(new_pose)
          visualization_marker=Marker()
          visualization_marker.header.frame_id='/map'
          visualization_marker.header.stamp=rospy.Time.now()  
          visualization_marker.type=1
          visualization_marker.scale.x=0.2
          visualization_marker.scale.y=0.2
          visualization_marker.scale.z=0.2
          visualization_marker.color.r=1.0
          visualization_marker.color.a=1.0
          visualization_marker.pose=new_pose.pose
          visualization_marker.lifetime=rospy.Duration(5.0)
          self.visualization_marker_pub.publish(visualization_marker)
      else:
        print(str(self.max_count)+"/"+str(self.total_count))

      r.sleep()


  def person_tracker_callback(self,data):
    if self.tf.frameExists(data.header.frame_id):

      #find how far the person is from you. 
      #If he is far enough move in his direction

      pose=self.tf.transformPose("/map",data)
      twist=Twist()
      #I assume that I do not care about the z dimension which is the person's height
      if math.hypot(pose.pose.position.x,pose.pose.position.y) < self.radius*3:
        self.max_count+=1
      self.total_count+=1
      #if robot is close enough don't move
      if math.hypot(pose.pose.position.x,pose.pose.position.y) > self.radius:
        #if pose is too different don't move
        #if @TODO

        #we are not at the goal time to move there
        move_base_navi=MoveBaseGoal()

        #we want to be slightly right and behind the person (Henrik)
        pose.pose.position.x-=0.2
        pose.pose.position.y-=0.2      
        move_base_navi.target_pose.header.frame_id='/map'
        move_base_navi.target_pose.header.stamp=rospy.Time.now()

        move_base_navi.target_pose.pose=pose.pose
        #move_base_navi.goal_id=1
        self.client.send_goal(move_base_navi)

        #rospy.logwarn(str(self.client.get_result()))

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
