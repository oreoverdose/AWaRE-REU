#!/usr/bin/env python
import sys
import rospy
import numpy as np
import trajectoryMatrices as tmx
import math as m
from std_msgs.msg import Float32MultiArray,Bool, Float32

class trajectory:
	def __init__(self):
		
		self.sys_out_sub = rospy.Subscriber("system_output",Float32MultiArray,self.sysOutCallback)
		
		self.sysOut_min_traj_pub = rospy.Publisher("sysOut_min_traj",Float32MultiArray,queue_size=5)
		
		self.done_pub = rospy.Publisher("done",Bool,queue_size=1)
		self.done_pub.publish(False)

		self.theta = 0		
		
		self.col = 0
		
		self.col_pos = 0
		self.traj_matrix = np.array([
		[-0.5500,0,0,0.0982,-0.0000],
	        [-0.5014,0.0000,0.0000,0.0961,0.0000],
	        [-0.4537,0.0000,0.0000,0.0948,-0.0000],
	        [-0.4050,0.0000,0.0000,0.1000,-0.0000],
	        [-0.3800,0,0.0000,0,0]]).transpose()
		self.initMatrix(self.col,self.traj_matrix)
	
	def initMatrix(self,col,traj_matrix):
		col = [[0 for x in range(4)] for y in range(len(traj_matrix[0]))]
		for i in range (len(traj_matrix[0])):
			col[i][0] = traj_matrix[0][i]
			col[i][1] = traj_matrix[1][i]
			col[i][2] = traj_matrix[3][i]*np.cos(traj_matrix[2][i])
			col[i][3] = traj_matrix[3][i]*np.sin(traj_matrix[2][i])
		self.col = col
	
	def sysOutCallback(self,sysOut):
		sysArr0 = np.asarray(list(sysOut.data))
		output = 0
		if not -0.05236<=sysArr0[2]<=0.05236:
			sysArr = np.array([[sysArr0[2]],[sysArr0[4]]])
			output = sysArr - np.array([[0],[0]])	
		
		else:
			traj = np.asarray(self.col[self.col_pos])
			sysArr = [sysArr0[0],
				  sysArr0[1],
				  sysArr0[3]*np.cos(sysArr0[2]),
				  sysArr0[3]*np.sin(sysArr0[2])]

			output = sysArr-traj
			print("~~~~")
			cont2 = False
			if self.col_pos < len(self.traj_matrix[0])-1:
				self.col_pos+=1
				if self.col_pos > len(self.traj_matrix[0])-1:
					self.col_pos = len(self.traj_matrix[0])-1
			elif self.col_pos >= len(self.traj_matrix[0])-1:
				cont = True
				for i in range(2):
					if not -0.05<output[i]<0.1:
						print("~~~error~~~")
						print(i)
						cont = False
				if cont:
					self.done_pub.publish(True)
			
			print("---traj---")
			print(traj)
		print("===2 x_hat===")
		print(output)
		self.sysOut_min_traj_pub.publish(data=output)
		

def main(args):
	rospy.init_node("sys_minus_trajectory",anonymous=True)
	tm = trajectory()
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
