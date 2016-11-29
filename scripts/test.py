'''
This test file takes state information and displays it
'''
from StateTree import StateTree
from Information import *
from CollisionChecking import *
error=0

#new las
def new_laser_scan_data(data):

	#2d pose estimate

	#figure out best velocity

	#send out new velocity command
	pass

def new_odometry(data):
	pass

def new_robot_information_data(data):
	pass

def predict_next_move():
	pass

#rospy.init_node('follower_node')
q=Information(1,2,0,1,0)
c=CollisionChecking(1)
st= StateTree(c)
st.plan()