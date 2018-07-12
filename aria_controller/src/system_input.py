#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist
from control_lift import GripperControl

class systemInput:
	
	def __init__(self):
		
		self.sys_min_traj_sub = rospy.Subscriber("sysOut_min_traj",Float32MultiArray,self.sysMinTrajCallback)
		
		self.done_sub = rospy.Subscriber("done",Bool,self.doneCallback)
		
		self.cmd_vel_pub = rospy.Publisher("/rosaria/cmd_vel",Twist,queue_size=10)
		
		self.sysArray_sub = rospy.Subscriber("sysArray",Float32MultiArray,self.sysCallback)
		
		self.sysArray=0
		
		self.prevV = 0.0884
		
		
	def doneCallback(self,done):
		if done:
			rospy.sleep(0.1)
			pub = Twist()
			pub.linear.x = 0
			pub.angular.z = 0
			self.cmd_vel_pub.publish(pub)
			rospy.sleep(0.1)
			gc = GripperControl()
			gc.raise_lift()
	
	def sysCallback(self,array):
		self.sysArray = array.data
		
		
	def sysMinTrajCallback(self,sysMinTraj):
		v=0
		w=0
		if len(sysMinTraj.data)==2:
			sysMinTraj = np.array([[sysMinTraj.data[0]],[sysMinTraj.data[1]]])
			rK = np.array([-1.3469,-2.1232])
			alpha = rK.dot(sysMinTraj)
			thetom1 = np.array([[1, 0.5],[0,1]]).dot(sysMinTraj)
			print()
			thetom2 = np.array([[0.125],[0.5]])*(alpha)
			print(thetom2)
			thetom = thetom1+thetom2
			print("thetom")
			print(thetom)
			w=thetom[1]
		if len(sysMinTraj.data)==4:
			mK = np.array([
				[-1.1973,0,-1.5931,0],
				[0,-1.1973,0,-1.5931]])
			sysMinTrajarr = np.asarray(list(sysMinTraj.data))
			virtualControl =mK.dot(sysMinTrajarr)
		
			#print("===3 [u1,u2]===")
			#print(virtualControl)
		
			#print("--cos(theta),sin(theta),velocity--")
			print("--velocity--")
			print(self.sysArray[2])
			
			u1 = virtualControl[0]
			u2 = virtualControl[1]
			cos = self.sysArray[0]
			sin = self.sysArray[1]
			v = self.sysArray[2]
		
			if -0.001<u2<-0.001:
				u2 = 0
			a = u1*cos + u2*sin
	
			if -0.0884<v<0.0884:
				v = 0.0884
			w = -u1*sin/v + u2*cos/v
		
			#print('===4. [a,w]===')
			#print([a,w])
			v = a*0.5 + self.prevV
		
			if v>1.4:
				v=1.4
			self.prevV = v
			if w>np.deg2rad(300):
				w=np.deg2rad(300)
			if w<np.deg2rad(-300):
				w=np.deg2rad(-300)
		
		pub = Twist()
		pub.linear.x = v
		pub.angular.z = -w
		print("===5. publish Twist ===")
		print(pub)
		self.cmd_vel_pub.publish(pub)
		

def main(args):

	rospy.init_node("sys_input",anonymous=True)
	si = systemInput()
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
