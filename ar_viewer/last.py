#!/usr/bin/env python

import cv2
import rospy, rospkg
import numpy as np
import math, time

import numpy as np
import collections

from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor
from cv_bridge import CvBridge

import sys
import os
import signal

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

pitchCut = 7


arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0
distance = 0
angle = 0

speed = 3.5
pubDrive = None

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(img_data):
	global cv_image
	global bridge
	cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

def callback(msg):
    global arData, pitch, distance, angle
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        (r,pitch,y) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        pitch = math.degrees(pitch)
        distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DZ"],2))
        try:
            angle = math.degrees(math.atan2(arData["DX"],arData["DZ"]))
        except:
            angle = 0

def drive(Angle, Speed): 
    global pubDrive

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pubDrive.publish(msg)


rospy.init_node('parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

pubDrive = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
pubImage = rospy.Publisher('movie', Image, queue_size=1)

rospy.sleep(1)
rate = rospy.Rate(10)

driveangle = 0

case = 0
#turn right angle +  turn left angle -
while not rospy.is_shutdown():
    
    while not cv_image.size == (640*480*3):
			continue
			
    frame = cv_image
		
    if cv2.waitKey(1) & 0xFF == 27:
        break
    
    if case == 0:
        driveangle = angle
        drive(driveangle,speed)
        
        if distance < 0.45:
            if abs(pitch) < pitchCut and abs(arData["DX"]) < 0.1:
                drive(0,0)
                print('finish')
                break
            if pitch > 0: #marker at left
                case = 1
                step1 = 0
                drive(0,0)
            else: #marker at right
                case = 3
                step1 = 0
                drive(0,0)
                
    if case == 1: #marker at left, positioning start
        driveangle = -30
        drive(driveangle, -speed)
        step1 += 1
        if step1 > 20:
            case = 2
            step1 = 0
            step2 = 0
            
    if case == 2 : #second positioning
        driveangle = 30
        drive(driveangle,-speed)
        step2 += 1
        if step2 > 10:
            case = 0
            step2 = 0
            drive(0,0)
            
    
    if case == 3: #marker at right, positioning start
        driveangle = 30
        drive(driveangle,-speed)
        step1 += 1
        if step1 > 20:
            case = 4
            step1 = 0
            step2 = 0
            
            
    if case == 4 : #second positioning
        driveangle = -30
        drive(driveangle,-speed)
        step2 += 1
        if step2 > 10:
            case = 0
            step2 = 0
            drive(0,0)
            
            
    text = 'x : ' + str(round(arData["DX"],2)) + '  z : ' + str(round(arData["DZ"],2))
    cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
    text = 'distance : ' + str(round(distance,2)) + '  pitch : ' + str(round(pitch,2)) 
    cv2.putText(frame, text, (20, 70), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
    text = 'driveangle : ' + str(round(driveangle,2))
    cv2.putText(frame, text, (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
    bgrframe = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
    pubImage.publish(bgrframe)
    #cv2.imshow("view", bgrframe)
    rate.sleep()

cv2.destroyAllWindows()
    
