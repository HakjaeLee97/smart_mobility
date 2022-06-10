#!/usr/bin/env python


from random import expovariate
import cv2, time
import numpy as np
import rospy, math, os, rospkg
import time
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers 
from tf.transformations import euler_from_quaternion
import math





bridge = CvBridge()
cv_image = np.empty(shape=[0])

Imu_msg = None

threshold_60 = 60
threshold_100 = 100
width_640 = 640
scan_width_200, scan_height_20 = 320, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10

vertical_430 = 370
row_begin_5 = (scan_height_20 - area_height_10) // 2 
row_end_15 = row_begin_5 + area_height_10 
pixel_threshold_160 = 0.35 * area_width_20 * area_height_10  
pixel_threshold_crossroad = 0.40 * area_width_20 * area_height_10 

#Threshold guide
#0.6 for nighttime, early morning
#0.65 for 14:00 
#0.75 for daytime
#0.55 for after noon

arData = {}
ar_tag_num = -1
ar_tag_pitch = 0
ar_tag_x = 0
ar_tag_z = 0

#================flags===================
flag_obstacle = 0
left_line_lost = 0
right_line_lost = 0
crossroad_count = 0
time_recorded = 0

tl_left = "" #left traffic light
tl_right = "" #right traffic light
tl_time = 0 # traffic change time


arData = {"id":-1, "DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0



#==============functions========================
def lidar_callback(obstacle_msg):
    global flag_obstacle
    if obstacle_msg.data == 1:
        flag_obstacle = 1
        
    else:
        flag_obstacle = 0
        

def tl_left_callback(tl_msg):
    global tl_left
    tl_left = tl_msg.data
    

def tl_right_callback(tl_msg):
    global tl_right
    tl_right = tl_msg.data

def tl_time_callback(tl_msg):
    global tl_time
    
    tl_time = tl_msg.data

def img_callback(img_data):
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")


def ar_tag_callback(msg):
    global arData
    global roll, pitch, yaw
    global ar_tag_z, ar_tag_x, ar_tag_pitch, ar_tag_num
    
    try:
        if msg.markers != []:
            
            for i in msg.markers:
                arData["id"] = i.id
                arData["DX"] = i.pose.pose.position.x
                arData["DY"] = i.pose.pose.position.y
                arData["DZ"] = i.pose.pose.position.z
                
                arData["AX"] = i.pose.pose.orientation.x
                arData["AY"] = i.pose.pose.orientation.y
                arData["AZ"] = i.pose.pose.orientation.z
                arData["AW"] = i.pose.pose.orientation.w
                ar_tag_num = arData["id"]
                (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
                roll = math.degrees(roll)
                ar_tag_pitch = math.degrees(pitch)
                yaw = math.degrees(yaw)
                
                ar_tag_x = arData["DX"]
                ar_tag_z = arData["DZ"]
                
    except:
        
        pass

def ar_tag_callback_bak(msg):
    
    global ar_tag_num
    ar_tag_num = msg.data
    if ar_tag_num != -1:
        print(ar_tag_num)
    
    
def imu_callback(data):
    global Imu_msg
    Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

def drive(Angle, Speed):
    global pub
    
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    
    pub.publish(msg)

def drive_stop():
    global pub
    
    msg = xycar_motor()
    msg.angle = 0
    msg.speed = 0

    pub.publish(msg)

def drive_sleep(sec):
    recorded_time = time.time()
    while (time.time() - recorded_time < sec):
        pass
    else:
        return

def start():
    global pub
    global cv_image
    global left_line_lost, right_line_lost
    global time_recorded
    global ar_tag_num
    global tl_left, tl_right, tl_time
    global ar_tag_pitch, ar_tag_z, ar_tag_x
    Speed = 5
    left, right = -1, -1
    crossroad_count = 0
    crossroad_flag = 0
    decelation_section_flag = 0
    decelation_section_count = 0
    go_left_flag = 0
    go_right_flag = 0
    ar_tag2_checked = 0
    parking_stage = 0
    

    rospy.init_node('auto_driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber("lidar_driver",Int32, lidar_callback,queue_size = 1)
    rospy.Subscriber("/Left_color",String,tl_left_callback)
    rospy.Subscriber("/Right_color",String, tl_right_callback)
    rospy.Subscriber("/time_count",Int64, tl_time_callback)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_tag_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
 

    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
 
    
    while not rospy.is_shutdown():
    
        while not cv_image.size == (640*480*3):
            continue
        
        frame = cv_image

        if cv2.waitKey(1) & 0xFF == 27:
            break

        


        if Imu_msg == None:
            
            continue
        else:
            (xycar_roll, xycar_pitch, xycar_yaw) = euler_from_quaternion(Imu_msg)
            



        roi = frame[vertical_430:vertical_430 + scan_height_20, :]
        roi_yellow = frame[vertical_430:vertical_430 + scan_height_20+20, :]
        roi_crossroad = frame[vertical_430:vertical_430 + scan_height_20+20, :]

        frame = cv2.rectangle(frame, (0, vertical_430-20), (width_640 - 1,
            vertical_430 + scan_height_20 -20), (255, 0, 0), 3) 
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv_yellow = cv2.cvtColor(roi_yellow, cv2.COLOR_BGR2HSV)
        hsv_crossroad = cv2.cvtColor(roi_crossroad, cv2.COLOR_BGR2HSV)

        lbound = np.array([0, 0, 120], dtype=np.uint8) 
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        lbound_yellow = np.array([20, 70, 70], dtype=np.uint8) 
        ubound_yellow = np.array([32, 255, 255], dtype=np.uint8)

        lbound_crossroad = np.array([0,0,100], dtype=np.uint8) 
        ubound_crossroad = np.array([131,255,255], dtype=np.uint8) 

        bin = cv2.inRange(hsv, lbound, ubound)
        bin_yellow = cv2.inRange(hsv_yellow, lbound_yellow, ubound_yellow)
        bin_crossroad = cv2.inRange(hsv_crossroad, lbound_crossroad, ubound_crossroad)

        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)
        view_yellow = cv2.cvtColor(bin_yellow, cv2.COLOR_GRAY2BGR)
        view_crossroad = cv2.cvtColor(bin_crossroad, cv2.COLOR_GRAY2BGR)

        left, right = -1, -1
        print "==============================================="
        
        for l in range(lmid_200, area_width_20, -1):
            area = bin[row_begin_5:row_end_15, l - area_width_20:l]
            if cv2.countNonZero(area) > pixel_threshold_160:
                left = l
                break

        for r in range(rmid_440, width_640): 
            area = bin[row_begin_5:row_end_15, r:r + area_width_20]
            if cv2.countNonZero(area) > pixel_threshold_160:
                right = r
                break
        
        for i in range(0,640):
            area_crossroad = bin_crossroad[row_begin_5:row_end_15+10, i:i + area_width_20]
            area_yellow = bin_yellow[row_begin_5:row_end_15+10, i:i + area_width_20]
            
            if (cv2.countNonZero(area_yellow) > pixel_threshold_crossroad):
                decelation_section_count +=1
            elif (cv2.countNonZero(area_crossroad) > pixel_threshold_crossroad):
            
                crossroad_count += 1
        #print "Crossroad count: ", crossroad_count

        if decelation_section_count >= 200 :
            print("slow region detected")
            print("decel count ",decelation_section_count)
            decelation_section_flag = 1
            crossroad_flag = 0
            decelation_section_count = 0

        elif crossroad_count >= 250:
            print("crossroad count: ",crossroad_count)
            print("crossroad detected")
            crossroad_flag = 1
            crossroad_count = 0
            decelation_section_flag = 0
            #single_crossroad_cross()    

        else:
            crossroad_count = 0
            crossroad_flag = 0
            decelation_section_count = 0
            decelation_section_flag = 0
            #print(crossroad_count)
        

        if left != -1:
            lsquare = cv2.rectangle(view,
                (left - area_width_20, row_begin_5),
                (left, row_end_15),
                (0, 255, 0), 3)
            left_line_lost = 0
        else:
            left_line_lost =1
            print("Lost left line")
            left = 0
        
        if right != -1:
            rsquare = cv2.rectangle(view,
                (right, row_begin_5),
                (right + area_width_20, row_end_15),
                (0, 255, 0), 3)
            right_line_lost = 0
        else:
            print("Lost right line")
            #right = 480
            right_line_lost = 1
        

        if ((left_line_lost)):
            center = right - 280
        elif ((right_line_lost)):
            center = left + 280
        else:    
            center = (right + left)/2
        if go_right_flag == 1:
            if (right - 280) > 0:
                center = right - 280
        elif go_left_flag ==1:
            center = left + 280

        #print "right: ", right
        #print "left: ", left
        #print "center:", center

        shift = center - 320
        Angle = shift/2
        if Angle < -50:
            Angle = -50
        if Angle > 50:
            Angle = 50
        stra = "Angle: "+repr(Angle)


        #==============Deceleration section=============

        if (ar_tag_num == 0)  or (decelation_section_flag == 1):
            if time_recorded != 1:
                print("slowing down")
                #print(ar_tag_num)
                record_time = time.time()
                time_recorded = 1
                
            if time.time()- record_time < 10: #for 10 secs
                Speed = 3
            else:
                print("return normal speed)")
                Speed = 5
                ar_tag_num = -1
                decelation_section_flag = 0
                time_recorded = 0
        else:
            pass
        if ar_tag_num != 1:
            print "Recognized ar tag id: ", ar_tag_num


        cv2.putText(frame, stra, (20,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255))

        #===========showing realtime video===========================
        cv2.imshow("origin", view)
        #cv2.imshow("view_crossroad", view_crossroad)   
        #cv2.imshow("view_yellow", view_yellow)
        
        
        #============2 way road ===================
        if ((ar_tag_num == 2 )and(ar_tag2_checked ==0) ):
            ar_tag2_checked = 1
            record_time = time.time()
            print "ar tag2 detected"
            print'================================='
            print tl_left, tl_right, tl_time

            if tl_left == "G":
                #if tl_time < 9:
                go_left_flag = 0
                crossroad_dir = "R"
                go_right_flag = 1
                print 'going right'
                
            elif tl_left == "Y":
                go_right_flag = 0
                go_left_flag = 1
                crossroad_dir = "L"
                print ' going left'
            
            elif tl_left == "R":
                go_left_flag = 1
                go_right_flag = 0
                crossroad_dir = "L"
                print 'going left'


        if ar_tag2_checked:
            if time.time() - record_time > 12:
                go_right_flag = 0
                go_left_flag = 0
            elif go_right_flag:
                print "going right"
            elif go_left_flag:
                print "going left"
            else:
                pass


        #==================parking=====================
        if (ar_tag_num == 6 ):
            print "parking stage :", parking_stage
            
            if parking_stage == 0:
                
                if (ar_tag_z > 2.0):
                    drive(Angle,3)
                    print "Now z degree is: ", ar_tag_z
                else:
                    time.sleep(2)
                    parking_stage = 2
                    aligned_yaw = xycar_yaw
                    
                    record_time = time.time()
            if parking_stage == 2:
                drive(50, -4)
                
                if (((xycar_yaw -aligned_yaw > 0.2) and ((time.time() - record_time > 0.2)))or  (time.time() - record_time > 0.5)):
                    parking_stage = 3
                    time.sleep(1)
                    record_time = time.time()
            if parking_stage == 3:
                drive(-50,-4)
                if (((-0.25 < xycar_yaw - aligned_yaw < 0.25) and ((time.time() - record_time) > 0.2)) or (time.time() - record_time)  > 0.4):
                    parking_stage = 4
                    ar_tag_z = 50
                    time.sleep(3)
            if parking_stage == 4:
                drive(ar_tag_pitch,3)
                if ar_tag_z < 1.5:
                    drive_stop()
                    print "Parking End"
                    parking_stage = 5
            if parking_stage ==5:
                break

        elif flag_obstacle == 1: # sudden obstacle recognized
            drive_stop()
            print "Emergency Stopping"



        elif crossroad_flag == 1:
            Angle = 0
            if (ar_tag2_checked != 1):
                if tl_right =="R" or tl_right =="Y":
                    drive(0,0) #stopping
                    print("crossroad stop")
                else: # green light
                    drive(Angle,Speed)
                    print "Go green light"
                    pass
            else:
                print "crossroad_dir is ", crossroad_dir
                print "Ar tag2 Recognized"
                print "go left flag :", go_left_flag
                print "go right flag :", go_right_flag
                if crossroad_dir == "L":
                    if tl_left =="R" or tl_left =="Y":
                        drive(0,0) #stopping
                        print("crossroad stop")
                    
                    else: # green light
                        drive(Angle,Speed)
                        print "Go green light"
                        pass  
                else:
                    if tl_right =="R" or tl_right =="Y":
                        drive(0,0) #stopping
                        print("crossroad stop")
                    else: # green light
                        drive(Angle,Speed)
                        print "Go green light"
                        pass  

        else:
            drive(Angle,Speed)
            pass
    
        
if __name__ == '__main__':

    
    start()
