#!/usr/bin/env python

##By: Sara Cooper
#Reads Skeleton message from Nuitrack software and converts the message type to PoseStamped so that it is published on /hand topic and read by Tiago, due to not being able to add the software plugin directly

import rospy
from body_tracker_msgs.msg import Skeleton
from geometry_msgs.msg import PoseStamped
import time
import datetime

hand_position = []
prev_hand_position = []
velocity = []
time = 0.1
movement_init = False
init_time = 0
finish_time = 0
init_position = []
counter = 0
track_time = 0
up_count = 0
forward_count = 0
object_grasp = 0
near_object = 0
move_start = 0
def callback(data):
        global hand_position
        hand_position= [data.joint_position_right_hand.x, data.joint_position_right_hand.y, data.joint_position_right_hand.z]

        

if __name__ == '__main__':

       rospy.init_node('skeleton_publisher', anonymous=True)  
       rospy.Subscriber("body_tracker/skeleton", Skeleton, callback)
       pub = rospy.Publisher('/hand', PoseStamped, queue_size=1)

       while not rospy.is_shutdown():
           rospy.sleep(time)

           if(hand_position):
                aruco_transform = PoseStamped()
                aruco_transform.pose.position.x = hand_position[0]
                aruco_transform.pose.position.y = hand_position[1]
                aruco_transform.pose.position.z = hand_position[2]
                aruco_transform.pose.orientation.x = 0
                aruco_transform.pose.orientation.y = 0
                aruco_transform.pose.orientation.z = 0
                aruco_transform.pose.orientation.w = 0            
                pub.publish(aruco_transform)

                
                   
                



