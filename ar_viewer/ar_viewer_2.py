#!/usr/bin/env python

import cv2
import rospy, math
import numpy as np
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
bridge = CvBridge()
cv_image = np.empty(shape=[0])

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/nvidia/xycar_ws/src/ar_viewer_jh/arparkar.avi',fourcc,30.0,(640,480))

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0
angle = 0
speed = 0
count = 0
prev_crash = 0
crash = 0
rcnt = 0
lcnt = 0
bcnt = 0

def callback(msg):
	global arData
	global count
	for i in msg.markers:
		arData["DX"] = i.pose.pose.position.x
		arData["DY"] = i.pose.pose.position.y
		arData["DZ"] = i.pose.pose.position.z
		arData["AX"] = i.pose.pose.orientation.x
		arData["AY"] = i.pose.pose.orientation.y
		arData["AZ"] = i.pose.pose.orientation.z
		arData["AW"] = i.pose.pose.orientation.w

def img_callback(img_data):
	global bridge
	global cv_image
	cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")	

def drive(Angle, Speed):
	global angle
	global speed
	global pub
	msg = xycar_motor()
	msg.angle = Angle
	msg.speed = Speed
	pub.publish(msg)

rospy.init_node('guide')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
drive(angle,speed)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	while not cv_image.size == (640*480*3):
		continue
	frame = cv_image

	if cv2.waitKey(1) & 0xFF == 27:
		break

	(roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
	roll = math.degrees(roll)
	pitch = math.degrees(pitch)
	yaw = math.degrees(yaw)
#	print(" roll : "+ str(roll))
	print("pitch : "+ str(pitch))
#	print(" yaw : "+ str(yaw))
#	print("x: "+ str(arData["DX"]))
#	print("y: "+ str(arData["DY"]))
	print("z: "+ str(arData["DZ"]))
	x = arData["DX"]
	z = arData["DZ"]
	
	if(z>0.5 and bcnt == 0 ):
		crash = 0
		if x>0.0:
			speed = 3
			angle = math.atan2(x,z)*180/3.14
		elif x<-0.0:
			speed = 3
			angle = math.atan2(x,z)*180/3.14
		
	else:	
		crash = 1
		if(prev_crash==0 and crash ==1):
			if(pitch>5):
				rcnt = 1
			elif(pitch<-5):
				lcnt = 1
			else:	
				speed = 0
				angle = 0	
		if(rcnt == 1):
			bcnt = bcnt+1
			if(bcnt<15):
				angle = -50
				speed = -3
			elif(bcnt<21):
				angle = 50
				speed = -3
			else:
				rcnt = 0
				bcnt = 0
				angle = 0
		elif(lcnt == 1):
			bcnt = bcnt+1
			if(bcnt<15):
				angle = 50
				speed = -3
			elif(bcnt<21):
				angle = -50
				speed = -3
			else:
				lcnt = 0
				bcnt = 0
				angle = 0
	prev_crash = crash
	drive(angle, speed)
	strp = str(pitch)
	strx = str(x)
	text = "yaw : " + strp + "   y : " + strx
	red = (0,0,255)
	cv2.putText(cv_image,text,(30,20),cv2.FONT_HERSHEY_SIMPLEX,0.4,red,1,cv2.LINE_AA)
	out.write(cv_image)	
	rate.sleep()	
