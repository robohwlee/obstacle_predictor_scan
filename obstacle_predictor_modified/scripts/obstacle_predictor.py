#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point32, Twist
from sensor_msgs.msg import LaserScan
try:
	from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
except:
	rospy.logerr('Failed to import ObstacleArrayMsg, ObstacleMsg.')

from cv2 import resize, calcOpticalFlowFarneback, GaussianBlur
from scipy.ndimage.filters import gaussian_filter, median_filter

from utils import *
import time
from copy import copy 

import tf
import math

class ObstaclePredictor:

	def __init__(self):

		# ROS parameters
		self.global_frame = rospy.get_param("/obstacle_predictor/global_frame_id")
		self.global_frame = self.global_frame[1:] \
			if self.global_frame[0]=='/' else self.global_frame
		self.base_frame = rospy.get_param("/obstacle_predictor/base_frame_id")
		self.obstacle_topic = rospy.get_param("/obstacle_predictor/obstacle_topic")
		self.movement_tol_max = rospy.get_param("/obstacle_predictor/movement_tol_max")
		self.movement_tol_min = rospy.get_param("/obstacle_predictor/movement_tol_min")
		self.prediction_horizon = rospy.get_param("/obstacle_predictor/prediction_horizon")
		self.timediff_tol = rospy.get_param("/obstacle_predictor/timediff_tol")
		self.flowdiff_tol = rospy.get_param("/obstacle_predictor/flowdiff_tol")
		self.window_size = rospy.get_param("/obstacle_predictor/window_size")

		self.num_r = 300 # number of ranges index in the polar image; num_r = int((r_max - r_min)/r_increment)

		# Initialize ros node
		rospy.init_node('obstacle_predictor', anonymous=True)

		# Scan buffer (10.07 added)
		self.scan_msg = None
		self.prev_scan_msg = None

		# Publisher and subscriber

		# scan tf listener
		self.listener = tf.TransformListener()

		# scan subscriber (10.07 added)
		# self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		self.scan_sub = rospy.Subscriber("/fined_scan", LaserScan, self.scanCallback)
		
		self.obstacle_pub = rospy.Publisher(self.obstacle_topic, ObstacleArrayMsg, queue_size=10)


	def spin(self):
		rospy.spin()


	def scanCallback(self, msg): # 10.08 add
		'''
		Save current scan to buffer
		'''
		self.scan_msg = self.reshapeScan(msg)
		if self.prev_scan_msg == None:
				self.prev_scan_msg = copy(self.scan_msg)
		
		self.predict_velocities_scan()

	def reshapeScan(self, msg): # 10.08 add
		'''
		Convert scan data to 2D image.
		'''
		num_theta = int((msg.angle_max - msg.angle_min)/msg.angle_increment)
		# 360; -pi to pi, 1 degree increment
		r_increment = 9.5/self.num_r # 0.05 for num_r = 190
		r_min = 0.5 # experimentally, max = 10 (m), min = 0.5(m)
		r_max = 10.0

		scan_image = np.zeros((self.num_r,num_theta)) # (num_r * 360) image
		
		for i in range(num_theta):
			j = int((msg.ranges[i] - r_min)/r_increment) # normalize range data to assign
			scan_image[j][i] = 100

		msg.ranges = scan_image[:,:]
		return msg


	def predict_velocities_scan(self):
		'''
		Compute optical flow of scan data to predict velocities of moving obstacles.
		Predicted velocities will be used for generating ObstacleArrayMsg for TebLocalPlannerROS.
		'''
		dt = self.scan_msg.header.stamp.to_sec() - self.prev_scan_msg.header.stamp.to_sec()
		if dt >0.01 and dt < self.timediff_tol: # skip opticalflow when dt is larger than self.timediff_tol (sec).
			# I1g, I2g = self.preprocess_scan_images()
			I1g = copy(self.prev_scan_msg.ranges)
			I2g = copy(self.scan_msg.ranges)

			flow = -calcOpticalFlowFarneback(I2g, I1g, None, 0.5, 3, self.window_size, 3, 5, 1.2, 0)
			#flow_ = calcOpticalFlowFarneback(I1g, I2g, None, 0.5, 3, self.window_size, 3, 5, 1.2, 0)
			
			# flowdiff = np.linalg.norm(flow - flow_, axis=2) > self.flowdiff_tol
			# flow[:,:,0][flowdiff] = 0
			# flow[:,:,1][flowdiff] = 0

			# convert coordinate from polar to cartesian
			flow_cartesian = self.transform_polar_to_cartesian(flow)
			
			# Generate and Publish ObstacleArrayMsg
			self.publish_obstacles_scan(flow_cartesian, dt)

			# Save current costmap to buffer
			self.prev_scan_msg = copy(self.scan_msg)


	def transform_polar_to_cartesian(self, flow):
		
		scan_cartesian = np.zeros((360, 4)) # angle 360, (x,y,dx,dy)
		
		for i in range(360):
			for j in range(self.num_r):
	   			if self.scan_msg.ranges[j][i] > 0:
					_theta = -math.pi + i * self.scan_msg.angle_increment # thetha = - pi + (i * angle_increment), theta(-pi, pi)
					_range = 0.5 + j * 0.05 # r = r_min + (j * r_increment) ,  ~ error range (0~r_increment), ranges(0.5, 10)
					_x = _range * math.cos(_theta)
					_y = _range * math.sin(_theta)
					
					# print("x,y")
					# print(_x,_y)
					
					scan_cartesian[i][0] = _x
					scan_cartesian[i][1] = _y
					scan_cartesian[i][2] = flow[j][i][1] * math.cos(_theta) - math.sin(_theta) * _range * flow[j][i][0]
					# dx = dr * cos(theta) - sin(theta) * r * dtheta
					scan_cartesian[i][3] = flow[j][i][1] * math.sin(_theta) + math.cos(_theta) * _range * flow[j][i][0]
					# dy = dr * sin(theta) + cos(theta) * r * dtheta

		return scan_cartesian


	def publish_obstacles_scan(self, flow, dt):
		'''
		Generate and publish ObstacleArrayMsg from flow vectors.
		'''
		
		scan_resolution = 20.0/self.num_r # scan_cartesian_one side(m)/index; resolution = 0.105 for index:190
		obstacle_vels = flow / dt * scan_resolution # dt = 0.0249~~
						
		# Generate obstacle_msg for TebLocalPlanner here.
		obstacle_msg = ObstacleArrayMsg()
		obstacle_msg.header.stamp = rospy.Time.now()
		obstacle_msg.header.frame_id = self.global_frame
		
		# get the scan sensor pose (= robot_pose)
		sensor_position, sensor_quaternion = self.listener.lookupTransform('/base_scan','/map', rospy.Time())
		sensor_euler = tf.transformations.euler_from_quaternion(sensor_quaternion) # return (roll, pitch, yaw)
		sensor_angle = - sensor_euler[2] # angle = yaw
		_cos = math.cos(sensor_angle)
		_sin = math.sin(sensor_angle)

		robot_pose = (sensor_position[0], sensor_position[1], sensor_angle)
		obstacle_speed = obstacle_vels[:,2:]
		obstacle_speed = np.linalg.norm(obstacle_speed, axis = 1)

		#mask_img = gaussian_filter(self.scan_msg.ranges.T, 1.0)
 		#obstacle_speed[mask_img < 40] = 0
		
		for i in range(obstacle_vels.shape[0]):
			# Add point obstacle to the message depend on obstacle speed
			if obstacle_speed[i] > 0 and obstacle_speed[i] < 1:
				m = sensor_position[0] + obstacle_vels[i][0]/3
				n = sensor_position[1] + obstacle_vels[i][1]/3
				# k = _cos*m - _sin*n
				# l = _sin*m + _cos*n
				# flow_vector_position = (
				# 	_cos*k + _sin*l,
				# 	- _sin*k + _cos*l
				# )
				flow_vector_position = (m,n)
				
				# flow_vector_position = (
				# 	_cos*sensor_position[0] + _sin*sensor_position[1] + obstacle_vels[i][0]/3,
				# 	_sin*sensor_position[0] + _cos*sensor_position[1] + obstacle_vels[i][1]/3
				# )
				# 첫 발행까지 시간소요 높음, 회전하면서 postion 동시에 변화

				# flow_vector_position = (
				# 	sensor_position[0] + (_cos*obstacle_vels[i][0] - _sin*obstacle_vels[i][1])/3,
				# 	sensor_position[1] + (_sin*obstacle_vels[i][0] + _cos*obstacle_vels[i][1])/3
				# )
				# 첫 발행까지 시간소요 높음, 회전하면서 postion 동시에 변화

				line_scale = (
					1. - np.exp(
						-np.linalg.norm([
							flow_vector_position[0] - robot_pose[0],
							flow_vector_position[1] - robot_pose[1]
						])/2.
					)
				)* self.prediction_horizon
				# print(flow_vector_position)
				obstacle_msg.obstacles.append(ObstacleMsg())
				obstacle_msg.obstacles[-1].id = len(obstacle_msg.obstacles)-1
				obstacle_msg.obstacles[-1].polygon.points = [Point32(), Point32()]
				obstacle_msg.obstacles[-1].polygon.points[0].x = flow_vector_position[0]
				obstacle_msg.obstacles[-1].polygon.points[0].y = flow_vector_position[1]
				obstacle_msg.obstacles[-1].polygon.points[0].z = 0
				obstacle_msg.obstacles[-1].polygon.points[1].x = flow_vector_position[0] + obstacle_vels[i][2]*line_scale/10
				obstacle_msg.obstacles[-1].polygon.points[1].y = flow_vector_position[1] + obstacle_vels[i][3]*line_scale/10
				obstacle_msg.obstacles[-1].polygon.points[1].z = 0

		self.obstacle_pub.publish(obstacle_msg)     # Publish predicted obstacles
		print("one msg published")

##  End of class ObstaclePredictor.

if __name__ == '__main__':

	try:
		node = ObstaclePredictor()
		node.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")

