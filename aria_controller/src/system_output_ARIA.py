#!/usr/bin/env python
import rospy
import sys
import numpy as np
import geometry_msgs.msg
import tf
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray,Bool,Float32


class systemOutput:

	def __init__(self):

		self.obj_pos = [0,-0.55]#[-0.1,0.54]#[-0.2,-0.512]#[0.55,0.229]#[0.4,-0.377]#[0.55,0.]#[0.3,-0.46]#[0.2,-0.512]#[0.1,0.54]#
		self.robo_rel_pos = 0
		self.gotRoboRelPos = False
		
		self.obj_found_sub = rospy.Subscriber("obj_found",Bool,self.foundCallback)
		
		self.odom_pub = rospy.Publisher("system_output",Float32MultiArray,queue_size = 10)
		
		self.done_sub = rospy.Subscriber("done",Bool,self.doneCallback)
		
		#get time... publish every 0.1 seconds
		self.time_log = rospy.get_time()
		
		#offset a couple of mm because gripper distance
		self.x = -0.55
		self.y = 0 
		
		self.theta = 0
		
	
	def doneCallback(self,done):
		rospy.signal_shutdown("doneeee!")
	
	def foundCallback(self, found):
		#get information from odom
		self.alpha = 0
		self.transform = 0
		self.odom_sub = rospy.Subscriber("/rosaria/pose",Odometry,self.odomCallback)
		self.theta_pub = rospy.Publisher("theta",Float32,queue_size = 10)
		self.sysArray_pub = rospy.Publisher("sysArray",Float32MultiArray,queue_size=10)
	
	def odomCallback(self, odom):
		position = odom.pose.pose.position
		orientation = odom.pose.pose.orientation
		
		
		if not self.gotRoboRelPos:
			#position where robot found object
			self.robo_rel_pos =[position.x,position.y]
			print("---robot starting position---")
			print(self.robo_rel_pos)
			self.gotRoboRelPos = True
			#angle between robo-world axis and object to robot axis
			self.alpha = math.atan2(self.obj_pos[1]-self.robo_rel_pos[1],self.obj_pos[0]-self.robo_rel_pos[0])
			print("---alpha---")
			print(self.alpha)
			self.transform = [
				[np.cos(self.alpha),np.sin(self.alpha)],
				[-np.sin(self.alpha),np.cos(self.alpha)]]
			##############FIX##############
			quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)

			#z-angle according to robot
			self.init_yaw = euler[2]
			################################
			
			
			
			self.theta = self.alpha
			self.theta_pub.publish(self.theta)

			rospy.sleep(2.5)
		
		###############FIX################
		quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		#z-angle according to robot
		yaw = euler[2]+self.init_yaw
		#################################
		
		
		self.now = rospy.get_time()
		if(self.now - self.time_log >= 0.5):
			print("---yaw---")
			print(yaw)
			self.theta = self.alpha - yaw
			vel = odom.twist.twist.linear.x
			self.sysArray_pub.publish(data=[np.cos(self.theta),np.sin(self.theta),vel])
			self.theta_pub.publish(self.theta)
			
			
			print("---real position---")
			print(position.x,position.y)
			
			#position in orginal axis relative to object
			orig_rel_pos = [[position.x-self.obj_pos[0]],[position.y-self.obj_pos[1]]]
			#get position in rotated axis relative to object
			prime_pos = np.dot(self.transform,orig_rel_pos)
			
			output = [
				prime_pos[0][0],
				prime_pos[1][0],
				self.theta,
				vel,
				odom.twist.twist.linear.z]
			print("======================================")
			print("===1 (x,y,theta,vel,rotVel===")
			print(output)
			self.odom_pub.publish(data = output)
			self.time_log = rospy.get_time()

def main(args):
	rospy.init_node("system_output",anonymous=True,disable_signals=True)
	so = systemOutput()
	
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
