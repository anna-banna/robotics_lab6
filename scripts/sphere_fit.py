#!/usr/bin/env python3
import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# set up the variables for 
raw_data = XYZarray()
A = []
B = []
params1 = SphereParams()
params2 = SphereParams()

# flag to make sure we receive data
data_received = False

# get the raw data 
def get_points(data): 
	global raw_data
	global data_received
	data_received = True
	raw_data = data
		
def make_arrays(raw_data): 
	global A
	global B
	A = []
	B = []
	for point in raw_data.points: 
		# do that math to set up A and B matrices 
		A.append([2*point.x, 2*point.y, 2*point.z, 1])
		B.append([point.x**2 + point.y**2 + point.z**2])
	A = np.array(A)
	B = np.array(B)
	
def doMath(A,B):
	global params
	# do the matrix math and manipulations
	ATA = np.matmul(A.T, A)
	ATB = np.matmul(A.T, B)
	P = np.matmul(np.linalg.inv(ATA), ATB)
	# assign variables to the P indices representing x,y,z
	x1 = P[0]
	y1 = P[1]
	z1 = P[2]
	# calculate r from P matrix values 
	r = math.sqrt(P[3] + x1**2 + y1**2 + z1**2)
	params1.xc = x1
	params1.yc = y1
	params1.zc = z1 
	params1.radius = r

def filterParams(params):
	# set the first input values
	fil_in_xc = params1.xc
	fil_in_yc = params1.yc
	fil_in_zc = params1.zc
	fil_in_rc = params1.radius
	# set the initial guess 
	fil_out_xc = -0.013679489493370056
	fil_out_yc = -0.016949649900197983
	fil_out_zc = 0.47584617137908936
	fil_out_rc = 0.05082815885543823
	# set the filter gain 
	fil_gain = 0.01
	
	# do the math for the next filter output values
	fil_out_xc = fil_gain * fil_in_xc + (1 - fil_gain)*fil_out_xc
	fil_out_yc = fil_gain * fil_in_yc + (1 - fil_gain)*fil_out_yc
	fil_out_zc = fil_gain * fil_in_zc + (1 - fil_gain)*fil_out_zc
	fil_out_rc = fil_gain * fil_in_rc + (1 - fil_gain)*fil_out_rc
	
	# set the next filter output values to the parameters message 
	params2.xc = fil_out_xc 
	params2.yc = fil_out_yc
	params2.zc = fil_out_zc 
	params2.radius = fil_out_rc
	

if __name__ == '__main__': 
	#define the node and subscribers and publishers 
	rospy.init_node('sphere_fit', anonymous = True)
	#define subscriber to receive cropped xyz point data from bag file 
	xyz_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_points)
	#define publisher for sphere parameters 
	xyz_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 1)
	
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown(): 
		if data_received:
			# call make arrays with raw data 
			make_arrays(raw_data)
			# call the function to do that math
			doMath(A,B)
			# call the function to filter the parameters 
			filterParams(params1)
			# publish the parameters 
			xyz_pub.publish(params2) 
		
		loop_rate.sleep() 
		
