#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers 
from tf.transformations import euler_from_quaternion
import math

distance = []
obstacle_msg = 0

arData = {"id":-1, "DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0


def callback(data):
    global distance
    distance = data.ranges


def obstacle_stop():
    global obstacle_msg
    obstacle_msg = 1
    pub.publish(obstacle_msg)
    print("published obstacle flag")

def ar_tag_callback(msg):
    global arData
    #print("ar tag detected")
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
                print(arData["id"])
                (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
                roll = math.degrees(roll)
                pitch = math.degrees(pitch)
                yaw = math.degrees(yaw)
                
                pub_artag.publish(arData["id"])
                pub_artag_pitch.publish(pitch)
                pub_artag_x.publish(arData["DX"])
                print "z is : ", arData["DZ"]
                pub_artag_z.publish(arData["DZ"])
    except:
        #print("markers is null")
        pass
 
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_tag_callback)

pub = rospy.Publisher('lidar_driver' , Int32 ,queue_size = 1)
pub_artag = rospy.Publisher('ar_tags',Int32 ,queue_size = 1)
pub_artag_pitch = rospy.Publisher('ar_tags_pitch', Float32 ,queue_size = 1) 
pub_artag_x = rospy.Publisher('ar_tags_x', Float32 ,queue_size = 1)
pub_artag_z = rospy.Publisher('ar_tags_z', Float32, queue_size = 1)

time.sleep(3) #ready to connect lidar


while not rospy.is_shutdown():
    ok = 0
    for degree in range(70,230):
        if distance[degree] <= 0.35:
            ok += 1
            #print "At degree ", degree ," Obstacle detected"
            #print "distancd is :", distance[degree]
        if ok > 3:
            obstacle_stop()
            break
    if ok <= 3:
        obstacle_msg = 0
        
        pub.publish(obstacle_msg)
	    #print("now go")
        continue
