#!/usr/bin/env python

import rospy, math
import rospkg
import numpy as np
import cv2, time
import math, os

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor
from cv_bridge import CvBridge

# image processing part
bridge = CvBridge()
cv_image = np.empty(shape = [0])

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/nvidia/xycar_ws/src/ar_viewer_yd/all.avi',fourcc,30.0,(640,480))

def img_callback(img_data):
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

# General Part : motor steering and displaying
motor_control = xycar_motor()

arData = {"DX" : 0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

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

rospy.init_node('guide')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1)

def motor_steer_right(pitch):
    global pub
    global motor_control
    motor_control.angle = round(pitch)
    motor_control.speed = 3
    
    #if pitch > 50 :
    #    motor_control.angle = 50
    #    motor_control.speed = 3
    #else : 
    #    motor_control.angle = 20
    #    motor_control.speed = 3

    pub.publish(motor_control)

def motor_steer_left(pitch):
    global pub
    global motor_control
    motor_control.angle = round(pitch)
    motor_control.speed = 3

    #if pitch < -40 :
    #    motor_control.angle = -40
    #    motor_control.speed = 3
    #else : 
    #    motor_control.angle = -20
    #    motor_control.speed = 3

    pub.publish(motor_control)

def motor_steer_straight():
    global pub
    global motor_control
    motor_control.angle = 0
    motor_control.speed = 3

    pub.publish(motor_control)

def motor_steer_stop():
    global pub
    global motor_control
    motor_control.angle = 0
    motor_control.speed = 0
    pub.publish(motor_control)


while not rospy.is_shutdown():
    global cv_image
    global pub    

    while not cv_image.size == (640*480*3):
        continue

    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

    roll = math.degrees(roll) 
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw) 
    
    
    distance = math.sqrt(math.pow(arData["DX"],2) + math.pow(arData["DY"], 2) + math.pow(arData["DZ"], 2)) 
    
    print("========================================")
    print("Distance is " + str(distance))
    print("Yaw is " + str(yaw))
    print("Roll : " + str(roll))
    print("Pitch : " + str(pitch))
    print("========================================")

    # which value are you gonna use?
    if distance > 0.5 :
        if pitch < -10:
            motor_steer_left(pitch)
#            motor_steer_stop()
        elif pitch >= 10: 
            motor_steer_right(pitch)
#            motor_steer_stop()
        else :
            motor_steer_straight()
#            motor_steer_stop()
    else :
        motor_steer_stop()
    

    # Number 1 : Preparation for position
    ratio_x = 1
    ratio_y = 1
    ratio_z = 1

    position_x = arData["DX"] * ratio_x
    position_y = arData["DY"] * ratio_y
    position_z = arData["DZ"] * ratio_z
    
    # Number 2 : Preparation for Orientation 
    # first of all, try with YAW 
    
    # Number 3 : Display both 
    # position, orientation display
 
    frame = cv_image

    font = cv2.FONT_HERSHEY_SIMPLEX
    org_1 = (0, 300)
    org_2 = (0, 400)
    fontScale = 1
    color = (255, 0 , 0) 
    thickness = 2
    view_with_value = cv2.putText(frame, 'Position of Tag is' + str(position_y) + '.',org_1,font,fontScale,color,thickness,cv2.LINE_AA)
    view_with_value = cv2.putText(frame, 'Orientation of Tags Pitch is' + str(pitch),org_2,font,fontScale,color,thickness,cv2.LINE_AA)
    cv2.imshow("view With value", view_with_value)
    out.write(cv_image)	
#cap.release()
#cv2.destroyAllWindows()

#   print(" roll : " + str(roll))
#   print("pitch : " + str(pitch))
#   print(" yaw  : " + str(yaw))

#   print("  x  : " + str(arData["DX"]))
#   print("  y  : " + str(arData["DY"]))
#   print("  z  : " + str(arData["DZ"]))
