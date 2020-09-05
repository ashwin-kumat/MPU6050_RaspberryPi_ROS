#!/usr/bin/env python

import serial
import rospy
import time
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry # Position, orientation(quat), linear-angular velo
#from pyquaternion import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField 

acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0
count = 0
q_x, q_y, q_z, q_w = 0.0, 0.0, 0.0, 0.0
q_x_init, q_y_init, q_z_init, q_w_init = 0.0, 0.0, 0.0, 0.0
calib_limit = 10
avg_window = 3
x_avg_arr = np.repeat(1.00,avg_window)
y_avg_arr = np.repeat(1.00,avg_window)
z_avg_arr = np.repeat(1.00,avg_window)
acc_threshold = 0.8
delta_t = 0.100



def call_back_fn(data):
	#print count
	global pitch, roll, yaw, quat_x, quat_y, quat_z, quat_w, acc_x, acc_y, acc_z, count
	global q_x_init, q_y_init, q_z_init, q_w_init, acc_x_init, acc_y_init, acc_z_init
	global q_x, q_y, q_z, q_w

	if (count<calib_limit):
		
		q_x_init =  data.orientation.x#/(count+1)
		q_y_init =  data.orientation.y#/(count+1) 
		q_z_init =  data.orientation.z#/(count+1)
		q_w_init =  data.orientation.w#/(count+1)

		acc_x_init = data.linear_acceleration.x
		acc_y_init = data.linear_acceleration.y
		acc_z_init = data.linear_acceleration.z

		#print 'q_x_init: ',q_x_init, q_y_init
		'''print q_x_fix
		print count
		print  '''
	q_x   = data.orientation.x 
	q_y   = data.orientation.y
	q_z   = data.orientation.z 
	q_w   = data.orientation.w
	#print q_x

	acc_x = data.linear_acceleration.x
	acc_y = data.linear_acceleration.y
	acc_z = data.linear_acceleration.z

    


	'''if abs(acc_x)>0.3:
		acc_x = acc_x
	else:
		acc_x = 0
	if abs(acc_y)>0.3:
		acc_y = acc_y
	else:
		acc_y = 0'''

	quaternion_arr = [q_x, q_y, q_z, q_w]
    
	(roll, pitch, yaw) = euler_from_quaternion(quaternion_arr)

def rpy_callback(data1):

	global filt_roll, filt_pitch, filt_yaw, init_roll, init_pitch, init_yaw

	filt_roll = math.degrees(data1.vector.x)
	filt_pitch = math.degrees(data1.vector.y)
	filt_yaw = math.degrees(data1.vector.z)

	if (filt_roll<0):
		filt_roll = (360 - abs(filt_roll))
		#print "here", filt_roll
	if (filt_pitch<0):
		filt_pitch = 360 - abs(filt_pitch)
	if (filt_yaw<0):
		filt_yaw = 360 - abs(filt_yaw)

	if (count<calib_limit):
		
		init_roll = filt_roll
		init_pitch = filt_pitch
		init_yaw = filt_yaw


def main():
	rospy.init_node('imu_pose_dist_vel_publisher', anonymous=True)
	rate = rospy.Rate(100) # 5hz
	rospy.loginfo("In main function")

	subs_quaternion = rospy.Subscriber('/imu/data', Imu, call_back_fn) # (topic_name, msg_type, call_back_fn)
	subs_rpy = rospy.Subscriber('/imu/rpy/filtered',Vector3Stamped, rpy_callback)
	pub_imu_odom = rospy.Publisher('/odom', Odometry, queue_size = 15)
	pub_RPY = rospy.Publisher('/RPY', Point, queue_size = 15)
	pub_rpy_deg = rospy.Publisher('/rpy/degrees',Vector3Stamped, queue_size = 15)

	imu_odom = Odometry()
	rpy = Point()
	rpy_deg = Vector3Stamped()

	dist_x, dist_y, dist_z, dist_trav = 0.0, 0.0, 0.0, 0.0
	vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
	
	global count, x_avg_arr, y_avg_arr, z_avg_arr
	max_arr_x,max_arr_y,max_arr_z = 0.5, 0.5, 0.5
	
	while not rospy.is_shutdown():

		if count>=calib_limit:


			### Euler angle Calculation Method
			 #, filt_roll
			
			theta_roll = (init_roll - filt_roll)  # Roll is rotation about  X axis
			theta_pitch = (init_pitch - filt_pitch)  # pitch is rotation about  Y axis
			theta_yaw = (init_yaw - filt_yaw)  # Yaw is rotation about  Z axis

			rot_yaw = np.array([[math.cos(math.radians(theta_yaw)),   math.sin(math.radians(theta_yaw)),   0],
								[-math.sin(math.radians(theta_yaw)),  math.cos(math.radians(theta_yaw)),   0],
								[0,                                                0,                      1]])
			rot_pitch = np.array([[math.cos(math.radians(theta_pitch)),       0,       -math.sin(math.radians(theta_pitch))],
								 [0,                                          1,                       0],
								 [math.sin(math.radians(theta_pitch)),        0,       math.cos(math.radians(theta_pitch))]])

			rot_roll = np.array([[1,                                     0,                              0],								 			
								 [0,               math.cos(math.radians(-theta_roll)), math.sin(math.radians(-theta_roll))],
								 [0,               -math.sin(math.radians(-theta_roll)),  math.cos(math.radians(-theta_roll))]])

			rotation_matrix = np.matmul(np.matmul(rot_roll, rot_pitch), rot_yaw).T #np.matmul(rotation_mat,body_frame_acc)
			#rotation_matrix = rot_yaw #np.matmul(np.matmul(rot_yaw,rot_pitch),rot_roll)

			body_frame_acc = np.array([acc_x, acc_y, acc_z])

			wf_acc = np.matmul(rotation_matrix,body_frame_acc) - np.array([0,0,acc_z_init])
			
			###### Simple Moving Average 

			x_avg_arr = np.delete((np.append(x_avg_arr,wf_acc[0])),0)
			x_avg = np.sum(x_avg_arr)/avg_window
			max_arr_xt = max(x_avg_arr)

			y_avg_arr = np.delete((np.append(y_avg_arr,wf_acc[1])),0)
			y_avg = np.sum(y_avg_arr)/avg_window
			max_arr_yt = max(y_avg_arr)

			z_avg_arr = np.delete((np.append(z_avg_arr,wf_acc[2])),0)
			z_avg = np.sum(z_avg_arr)/avg_window
			max_arr_zt = max(z_avg_arr)

			#### Storing max accelerations(for refrence)

			if abs(max_arr_xt)>abs(max_arr_x):
				max_arr_x = max_arr_xt
				#print max_arr_x
			if abs(max_arr_yt)>abs(max_arr_y):
				max_arr_y = max_arr_yt
			if abs(max_arr_zt)>abs(max_arr_z):
				max_arr_z = max_arr_zt
			 
			###### Direct Euler component addition method

			'''ax = -acc_z_init*math.sin(math.radians(-filt_roll))
			ay = -acc_z_init*math.cos(math.radians(-filt_roll))*math.sin(math.radians(filt_pitch))
			az = -acc_z_init*math.cos(math.radians(-filt_roll))*math.cos(math.radians(filt_pitch))'''
			#print math.cos(math.radians(filt_roll))*math.cos(math.radians(filt_pitch))

			if abs(x_avg)>acc_threshold:
				vel_x = abs(x_avg)*delta_t
				vel_y = abs(y_avg)*delta_t
			#vel_z = (acc_z+)*delta_t			
				dist_x += abs(x_avg)*delta_t**2
				dist_y += abs(y_avg)*delta_t**2
			#dist_z += vel_z*delta_t
				dist_trav = math.sqrt(dist_x**2 + dist_y**2)



			##### Data publishing to topics

			rpy.x = math.degrees(roll)
			rpy.y = math.degrees(pitch)
			rpy.z = math.degrees(yaw)

			rpy_deg.vector.x = filt_roll
			rpy_deg.vector.y = filt_pitch
			rpy_deg.vector.z = filt_yaw

			imu_odom.pose.pose.position.x = dist_x
			imu_odom.pose.pose.position.y = dist_y
			#imu_odom.pose.pose.position.z = dist_z

			imu_odom.twist.twist.linear.x = vel_x
			imu_odom.twist.twist.linear.y = vel_y
			#imu_odom.twist.twist.linear.z = vel_z

			pub_RPY.publish(rpy)
			pub_imu_odom.publish(imu_odom)
			pub_rpy_deg.publish(rpy_deg)
			

			if (count%50 == 0):
				print count

			#print "Body frame Acc  :", body_frame_acc
			#print "Orientation Init/World ", 			
			#print "Quat Init/World :", q_x_init, q_y_init, q_z_init, q_w_init
			#print "Quat current    :", q_x, q_y, q_z, q_w

				print "Rotation angles", np.around(np.array([theta_roll, theta_pitch, theta_yaw]), decimals=3)
				print "Euler Init/World RPY:", np.around(np.array([init_roll, init_pitch, init_yaw]), decimals=3)  #math.degrees(world_roll), math.degrees(world_pitch), math.degrees(world_yaw)
				print "Euler Current RPY   :", np.around(np.array([filt_roll, filt_pitch, filt_yaw]), decimals=3)#math.degrees(current_roll), math.degrees(current_pitch), math.degrees(current_yaw)

				#print "Rotation Matrix Euler:"
				#print np.around(rotation_matrix, decimals=3)
				#print "World frame Acc :", np.around(wf_acc, decimals=3)
				print "IMU Acceleration:", np.array([acc_x, acc_y, acc_z])
				print "Moving avg acc: ", np.around(np.array([x_avg,y_avg,z_avg]), decimals=3)
				print "Maximum Acceleration: ", np.around(np.array([max_arr_x,max_arr_y,max_arr_z]), decimals=4)
				print "Initial Acceleration", np.array([acc_x_init, acc_y_init, acc_z_init])
				print "Distance estimate", np.around(np.array([dist_x, dist_y, dist_trav]), decimals=4)
				#print np.around(x_avg_arr, decimals=3)
				#print "Component Acceleration: ", np.array([ax,ay,az])

				print ""


		rate.sleep()

		#print acc
		#print type(acc)
		#print split_data[-1]
		#print ""
		count += 1 
		
		
	rate.sleep()
	rospy.loginfo("Node is shutting down")

if __name__ == '__main__':
	main()
