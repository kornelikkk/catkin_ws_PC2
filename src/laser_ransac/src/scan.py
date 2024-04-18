#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np
from matplotlib import pyplot as plt

from sklearn import datasets, linear_model
from math import sin, cos, pi, atan2

from geometry_msgs.msg import Twist

def line_visual(line_X, line_y_ransac):
	marker = Marker()
	marker.header.frame_id = "my_fixed_frame"
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD


    # marker scale
	marker.scale.x = 0.3
	marker.scale.y = 0.3
	marker.scale.z = 0.3

    # marker color
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 1.0

	# marker orientaiton
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	# marker position
	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.0

    # marker line points
	marker.points = []
    # first point
	first_line_point = Point()
	first_line_point.x = line_X[0]
	first_line_point.y = line_y_ransac[0] 
	first_line_point.z = 0.0
	marker.points.append(first_line_point)
    # second point
	second_line_point = Point()
	second_line_point.x = line_X[-1]
	second_line_point.y = line_y_ransac[-1] 
	second_line_point.z = 0.0
	marker.points.append(second_line_point)

    # Publish the Marker
	pub_line_min_dist.publish(marker)


def callback(msg):
	
	ranges_edge = msg.ranges

	X = []
	y = []
	start_phi = 0 # стартовый угол в градусах
	finish_phi = 180
	angle = int(start_phi*210/360)*360/210
	
	min_value = 100

	for point in ranges_edge[int(finish_phi*210/360):int(start_phi*210/360):-1]:
		if point > 0.05:
			theta = angle * pi/180.0
			X.append([point*cos(theta)])
			y.append([point*sin(theta)])
			if point < min_value:
				min_value = point
		angle += 360/210

		#print(angle)


	#smallest = min(ranges_edge[int(start_phi*210/360):int(finish_phi*210/360)])
	count = ranges_edge[int(start_phi*210/360):int(finish_phi*210/360)].count(min_value)
	
	print(f"The smallest number is {min_value}, it occurs {count} times.")


	X = np.array(X)
	y = np.array(y)

	# Fit line using all data
	lr = linear_model.LinearRegression()
	lr.fit(X, y)

	# Robustly fit linear model with RANSAC algorithm
	ransac = linear_model.RANSACRegressor()
	ransac.fit(X, y)
	inlier_mask = ransac.inlier_mask_
	outlier_mask = np.logical_not(inlier_mask)

	# Predict data of estimated models
	line_X = np.arange(X.min(), X.max())[:, np.newaxis]
	line_y = lr.predict(line_X)
	line_y_ransac = ransac.predict(line_X)

	#print(line_X, "Y", line_y_ransac)
	radian_robot = np.arctan2((line_X[-1] - line_X[0]), (line_y_ransac[-1] - line_y_ransac[0]))
	#print("Line X", line_X[0], "Line y", line_y_ransac[0], "END X", line_X[-1], "END y", line_y_ransac[-1])
	print("atan2 = ",  radian_robot)
	line_visual(line_X, line_y_ransac)

	command = Twist()            
	command.linear.x = 0.12
	
	# for left side
	command.angular.z = ((1.62 - radian_robot)*-1.2) - (0.40-min_value)*5
	
	# for right side
	#command.angular.z = ((1.62 - radian_robot)*-1.2) + (0.30-min_value)*5

	pub.publish(command)







rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub_line_min_dist = rospy.Publisher('~line_min_dist', Marker, queue_size=1)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.spin()


