#!/usr/bin/env python

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

global now
now = 0

def callback(data):
	global now
	orientation = data.pose.pose.orientation
	#quaternion=(    orientation.x,
	#		orientation.y,
	#		orientation.z,
	#		orientation.w)
	#euler = tf.transformations.euler_from_quaternion(quaternion)
	if (rospy.get_time()-now >= 0.1):
		rospy.loginfo(data.twist.twist.linear.x)
		now = rospy.get_time()
	
def callback2(data):
	rospy.loginfo("meoooow")

def listener():
	rospy.init_node('listener', anonymous=True)
	global now
	now = rospy.get_time()
	rospy.Subscriber('/Rosaria/odom', Odometry, callback)
	
	rospy.Subscriber('/Rosaria/cmd_vel', Twist, callback2)
	
	rospy.spin()
	
if __name__ == '__main__':
	listener()
