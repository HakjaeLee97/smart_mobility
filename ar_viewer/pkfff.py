#!/usr/bin/env python

import rospy, rospkg
import numpy as np
import math, time

import numpy as np
import collections

from xycar_motor.msg import xycar_motor

import sys
import os
import signal

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

pitchCut = 7
zCut = 0.5
finzCut = 0.4
xCut = 0.1

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0

distance = 0
lrstatus = 0
pre_lr = 0

speed = 3
sleep = 0.1
pubDrive = None

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
            angle = math.degrees(math.atan(arData["DZ"]/arData["DX"]))
        except:
            angle = 0
        #print('pitch : ' + str(pitch))
        #print('dx : ' + str(arData["DX"]))
        #print('dz : ' + str(arData["DZ"]))

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pubDrive

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pubDrive.publish(msg)
    #print('driving...')

def approach():
    global arData, speed, sleep, angle, lrstatus
    if pitch > 0 : #turn right
        lrstatus = 1
        while abs(pitch) > pitchCut and arData["DZ"] > zCut and abs(angle) > 10:
            print('turn right')
            drive(10,speed)
            rospy.sleep(sleep)
        #if abs(pitch) < pitchCut:
        #    while arData["DZ"] > zCut:
        #        print('straight')
        #        drive(0,speed)
        #        rospy.sleep(sleep)
    else:
        lrstatus = 0
        while abs(pitch) > pitchCut  and arData["DZ"] > zCut and abs(angle) > 10:
            drive(-10,speed)
            print('turn left')
            rospy.sleep(sleep)
        #if abs(pitch) < pitchCut:
        #    while arData["DZ"] > zCut:
        #        print('straight')
        #        drive(0,speed)
        #        rospy.sleep(sleep)

def goback():
    global sleep, lrstatus, pre_lr, angle
    if arData["DX"] < 0:
        if lrstatus == 0:
            while abs(pitch) > pitchCut:
                print('back left')
                drive(-25,-speed)  
                rospy.sleep(sleep)
            if abs(pitch) < pitchCut and abs(angle) > 80 :
                while arData["DZ"] > finzCut:
                    print('gbstraight')
                    drive(0,speed)
                    rospy.sleep(sleep)
                while True:
                    print('over')
        while distance < 1.3: #abs(pitch) < 50 and
            print('back right')
            #if lrstatus * pre_lr = -1:
            #    break 
            drive(30,-speed)
            #pre_lr=lrstatus
            rospy.sleep(sleep)
    else:
        #pre_lr = lrstatus
        if lrstatus == 1:
            while abs(pitch) > pitchCut:
                print('back right')
                drive(25,-speed)  
                rospy.sleep(sleep)
            if abs(pitch) < pitchCut and abs(angle) > 80:
                while arData["DZ"] > zCut:
                    print('gbstraight')
                    drive(0,speed)
                    rospy.sleep(sleep)
                while True:
                    print('over')
        while distance < 1.3: #abs(pitch) < 50 and 
            print('back left')
            drive(-30,-speed)  
            rospy.sleep(sleep)    

def judgement():
    if abs(arData["DX"]) < xCut and abs(pitch) < pitchCut:
        while arData["DZ"] > finzCut:
            print('final straight')
            drive(0,speed)
            rospy.sleep(sleep)
        drive(0,0)
        print("finished")
        rospy.sleep(10)
    else:
        rospy.sleep(0.1)
        print('goback')
        goback()
        

rospy.init_node('parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
pubDrive = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

rospy.sleep(1)
while not rospy.is_shutdown():
    
    x = arData["DX"]
    if pitch == 0: #arData["DX"] == x or 
        #print('sleeping')
        continue
    z = arData["DZ"]
    

    #print('angle : ' + str(angle) + '\n' + 'pitch : ' + str(pitch))

    approach()
    judgement()
    
    
cv2.destroyAllWindows()
