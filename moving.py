#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest('moving_tur')
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils
import math
import random
k=None
move=None
pub=None
extRight=None
angle_range=None
PI = 3.1415926535897
Dist=None
COLORS = ['red','green','blue']
RANGES =[([90,0,0], [255, 100, 100]),([0,90,0], [30, 255, 30]),([0,0,90], [80, 80, 255])]
color = None
find = False
colored = False
angle = None
Done = False
done_searching=False
got_dist = False
def callback_ass1(msg):
	global move,Done,sub,pub
	if msg.ranges[0]>0.5:
		move.linear.x = 0.5
		move.angular.z = 0
		pub.publish(move)
	else:
		move.linear.x = 0
		move.angular.z = 0
		pub.publish(move)
	

def ass1():
	global move,pub
	rospy.init_node('obst')
	sub = rospy.Subscriber('/scan', LaserScan, callback_ass1)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	move = Twist()
	rospy.spin()

def ass2():
	global pub, move
	angle = input("insert angle:")
	angle = angle*2*PI/360
	rospy.init_node('rotating', anonymous=True)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	move = Twist()
	move.linear.x =0
	move.linear.y =0
	move.linear.z =0
	move.angular.x =0
	move.angular.y =0
	speed = 5*2*PI/360
	move.angular.z=-speed
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	while current_angle<angle:
		pub.publish(move)
		t1=rospy.Time.now().to_sec()
		current_angle = speed*(t1-t0)
	move.angular.z=0
	pub.publish(move)
	rospy.spin()


def find_dist(msg):
	global angle_range,Dist,got_dist
	b=angle_range[0]
	a=	angle_range[1]
	if(angle_range[0]<angle_range[1]):
		a=angle_range[0]
		b=	angle_range[1]
	min_dist = 7000
	for x in range(a,b):
		if(min_dist>msg.ranges[x]):
			min_dist = msg.ranges[x]
	if min_dist != 7000:
		Dist = min_dist
	got_dist = True

def calc_angle_range(Start,End):
	Xstart=Start[0]-400
	Xend=End[0]-400
	return (int(359-math.ceil(0.1*Xstart))%360,int(359-0.1*Xend)%360)

def get_laser_distance():
	sub = rospy.Subscriber('/scan', LaserScan, find_dist)



def get_distance(img):
       #calculate x and y
	global angle_range,color,COLORS,RANGES
	count = COLORS.index(color)
	bridge=CvBridge()
	cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
	(lower, upper) = RANGES[count]
	lower = np.array(lower, dtype = "uint8")
    	upper = np.array(upper, dtype = "uint8")
	mask = cv2.inRange(cv_image, lower, upper)
	output1 = cv2.bitwise_and(cv_image, cv_image, mask = mask)
	output1[np.where((output1 != [0,0,0]).all(axis = 2))] = [255,255,255]
	gray = cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	# threshold the image, then perform a series of erosions +
	# dilations to remove any small regions of noise
	thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY)[1]
	thresh = cv2.erode(thresh, None, iterations=2)
	thresh = cv2.dilate(thresh, None, iterations=2)
	
	# find contours in thresholded image, then grab the largest
	# one

	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	c = max(cnts, key=cv2.contourArea)
	StartPoint = tuple(c[c[:, :, 0].argmin()][0])
	EndPoint = tuple(c[c[:, :, 0].argmax()][0])
	
	angle_range=calc_angle_range(StartPoint,EndPoint)
	get_laser_distance()
	

"""
	cv2.drawContours(cv_image, [c], -1, (0, 255, 255), 2)
	cv2.circle(cv_image, StartPoint, 8, (0, 0, 255), -1)
	cv2.circle(cv_image, EndPoint, 8, (0, 255, 0), -1)
	cv2.subtract(255, gray) 
	new_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
	cv2.imshow("Image", new_img)
	cv2.waitKey(0)
"""


def ass3():
	global k, Dist,color,Done,got_dist
	color=raw_input("Enter color:")
	rospy.init_node('im')
	k=rospy.Subscriber('/camera/image_raw',Image,get_distance)
	while not got_dist:
		pass
	print(Dist)
	rospy.signal_shutdown("reason")

def move_with_angle(angle):
	global pub,sub, move
	angle = angle*2*PI/360
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	move = Twist()
	move.linear.x =0
	move.linear.y =0
	move.linear.z =0
	move.angular.x =0
	move.angular.y =0
	speed = 8*2*PI/360
	move.angular.z=-speed
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	while current_angle<angle:
		pub.publish(move)
		t1=rospy.Time.now().to_sec()
		current_angle = speed*(t1-t0)
	move.angular.z=0
	pub.publish(move)
	angle = None
	sub=rospy.Subscriber('/scan', LaserScan, callback_ass4)
	while not Done:
		pass
	

def searcher(img):
	global angle_range,color,COLORS,RANGES, find,angle,sub,done_searching
	count = COLORS.index(color)
	bridge=CvBridge()
	cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
	(lower, upper) = RANGES[count]
	lower = np.array(lower, dtype = "uint8")
    	upper = np.array(upper, dtype = "uint8")
	mask = cv2.inRange(cv_image, lower, upper)
	output1 = cv2.bitwise_and(cv_image, cv_image, mask = mask)
	output1[np.where((output1 != [0,0,0]).all(axis = 2))] = [255,255,255]
	gray = cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	# threshold the image, then perform a series of erosions +
	# dilations to remove any small regions of noise
	thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY)[1]
	thresh = cv2.erode(thresh, None, iterations=2)
	thresh = cv2.dilate(thresh, None, iterations=2)
	
	# find contours in thresholded image, then grab the largest
	# one

	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	if cnts:
		c = max(cnts, key=cv2.contourArea)
		StartPoint = tuple(c[c[:, :, 0].argmin()][0])
		EndPoint = tuple(c[c[:, :, 0].argmax()][0])
		angle_range=calc_angle_range(StartPoint,EndPoint)
		print(angle_range)
		if abs(angle_range[1]-angle_range[0])>10:
			new_angle1 = angle_range[1]
			new_angle2 = angle_range[0]
			if 0 <= new_angle1 <= 40:
				new_angle1 = new_angle1 + 360
			if 0 <= new_angle2 <= 40:
				new_angle2 = new_angle2 + 360
			angle = 360-(((new_angle1 + new_angle2)/2)%360)
			find = True
			print("found")
		else:
			#print("found but keep searching")
			angle=90
	elif not find:
		angle=90
		print("bla")
	done_searching = True
	#rospy.signal_shutdown("reason")


def callback_ass4(msg):
	global move,Done,sub,pub,find
	if ((not find) and msg.ranges[0]>=1) or (find and msg.ranges[0]>0.5):
		move.linear.x = 0.5
		move.angular.z = 0
		pub.publish(move)
	else:
		move.linear.x = 0
		move.angular.z = 0
		pub.publish(move)
		Done = True

def ass4():
	global color,colored,pub,sub,move,find,angle,Done,done_searching
	if not colored:
		color=raw_input("Enter color:")
		colored = True
		rospy.init_node('searcher')
	while True:
		for x in range(30):
			sub=rospy.Subscriber('/camera/image_raw',Image,searcher)
			while not done_searching:
				pass
			done_searching = False
			sub.unregister()
		move = Twist()
		if not angle is None:
			move_with_angle(angle)
			angle = None
			sub.unregister()
		if find and Done:
			break
		Done = False 
	rospy.signal_shutdown("reason")
		#sub = rospy.Subscriber('/scan', LaserScan, callback_ass4)
		#pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		

funcs = (ass1,ass2,ass3,ass4)

def main():
	global funcs
	print("choose an option:\n1.Move forward\n2.Turn around\n3.Distance to object with color X\n4.Find object with color X")
	option = int(input())
	funcs[option-1]()
if __name__=='__main__':
	main()
