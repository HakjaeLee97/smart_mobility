#!/usr/bin/env python

import rospy, math

from xycar_motor.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

motor_msg = xycar_motor()

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0

def callback(msg):
	global arData
	for i in msg.markers:
		arData["DX"] = i.pose.pose.position.x
		arData["DY"] = i.pose.pose.position.y
		arData["DZ"] = i.pose.pose.position.z
		arData["AX"] = i.pose.pose.orientation.x
		arData["AY"] = i.pose.pose.orientation.y
		arData["AZ"] = i.pose.pose.orientation.z
		arData["AW"] = i.pose.pose.orientation.w

def drive_left(msg):
	global motor_msg
	motor_msg.speed = 2
	motor_msg.angle = msg
	pub.publish(motor_msg)

def drive_right(msg):
	global motor_msg
	motor_msg.speed = 2
	motor_msg.angle = msg
	pub.publish(motor_msg)

def drive_stop():
	global motor_msg
	motor_msg.speed = 0
	motor_msg.angle = 0
	pub.publish(motor_msg)

rospy.init_node('guide')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
	(roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"])) 

	roll = math.degrees(roll)
	pitch = math.degrees(pitch)
	yaw = math.degrees(yaw)

	
	print(" roll : "+ str(roll))
	print("pitch : "+ str(pitch))
	print(" yaw : "+ str(yaw))

	print(" x : "+ str(arData["DX"]))
	print(" y : "+ str(arData["DY"]))
	print(" z : "+ str(arData["DZ"]))

	if yaw>0 or yaw <179:
		drive_left(10)

	if yaw<0 or yaw >-179:
		drive_right(-10)

	if yaw>179 or yaw <-179:
		drive_stop()

