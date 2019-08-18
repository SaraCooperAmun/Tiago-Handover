#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from actionlib import SimpleActionClient
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from std_msgs.msg import String
import time
from datetime import datetime
import rosbag
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance, JointTrajectoryControllerState
from actionlib import SimpleActionClient, GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import MoveGroupActionFeedback
from geometry_msgs.msg import PoseStamped
import yaml
import io
from geometry_msgs.msg import Point
#Adjust the participant id and the rosbag where the data is saved accordingly
patient_id = '2019001' 
bag  = rosbag.Bag('/home/pal/sara_ws/patient_data/hand'+patient_id+'.bag', 'w')
bag.close()

def get_status_string(status_code):
    return GoalStatus.to_string(status_code)


#Functions to write data to rosbags: position() saves a Point() message with hand coordinates; velocity_h saves a Point() message
#with hand velocity; and gesture_h saves a String() message with any relevant detected gestures

def position(data):
       
          bag  = rosbag.Bag('/home/pal/sara_ws/patient_data/hand'+patient_id+'.bag', 'a')
          hand_c = Point()
          hand_c.x = data[0]
          hand_c.y = data[1]
          hand_c.z = data[2]
          bag.write('handpos', hand_c)
          bag.close()

def velocity_h(data):
       
          bag  = rosbag.Bag('/home/pal/sara_ws/patient_data/hand'+patient_id+'.bag', 'a')
          hand_c = Point()
          hand_c.x = data[0]
          hand_c.y = data[1]
          hand_c.z = data[2]
          bag.write('handvel', hand_c)
          bag.close()

def gesture_h(data):
       
          bag  = rosbag.Bag('/home/pal/sara_ws/patient_data/hand'+patient_id+'.bag', 'a')
          s = String()
          s.data = data
          bag.write('handges', s)
          bag.close()



arm_joints = []
def arm_state(msg):
     global arm_joints 
     arm_joints = msg

moveit_status= 0
def moveit_state(msg):
     global moveit_status
     moveit_status = msg.status.status


#Initialize all parameters
hand_position = []
prev_hand_position = []
velocity = []
time_hand = 0.1
movement_init = False
init_time = 0
finish_time = 0
init_position = []
counter = 0
track_time = 0
up_count = 0
forward_count = 0
down_count = 0
back_count = 0
object_grasped = 0
near_object = 0
move_start = 0
hand_up = False
hand_forward = False
grasped = False
hand_back = False
hand_down = False


#Main function to process hand data
def hand_process():
        global prev_hand_position 
	global velocity 
	global time 
	global movement_init
	global init_time 
	global finish_time 
	global init_position 
	global counter
	global track_time
	global up_count 
	global forward_count 
	global object_grasped
	global near_object 
	global move_start 
        global hand_up
        global hand_forward
        global grasped
        global down_count
        global back_count
        if(len(hand_position)!=0):
        
                if(prev_hand_position):
                        #Compute velocity of the hand by difference in position in a time-frame of 0.1 s
                	velocity = [abs(hand_position[0] - prev_hand_position[0])/time_hand, abs(hand_position[1] - prev_hand_position[1])/time_hand, abs(hand_position[2] - prev_hand_position[2])/time_hand]
                        
                        #Determine if hand is at reference position (button press) and stable. It must be still (velocity less than 0.01 m/s) for 5 consecutive readings)
                        if(len(init_position)==0):
           
		                if(counter < 5):
		                      if(max(velocity) < 0.01): 
		                           counter+=1
		                      else:
		                           counter = 0
                                      print("not stabilised")
		                else:
		                     init_position = [hand_position[0], hand_position[1], hand_position[2]]#Save the initial hand position to be taken as reference
		                     
		                     print(init_position, "hand stabilised")
                                     write_hand("Hand stabilisied at position " + str(init_position))

                        


                        else:
                         

                                #Once the hand is stable, start detecting gestures.  Taking into account the reference system for the Astra camera:
                                         # hand moved forward: decrease in y
                                         # hand moved back: increase in y
                                         # hand moved up: increase in z
                                         # hand moved down: decrease in z

                                #If the hand position is 0.12 m less in y than the reference position for 5 consecutive readings, the hand has moved forward
		                if(hand_position[1] < init_position[1] and abs(hand_position[1] - init_position[1]) > 0.12):
                                    forward_count +=1
                                    if(forward_count == 5):
 
		                   	 print "Hand moved forward" 
                                         write_hand("Hand moved forward")
                                         gesture_h("Person started moving")
                                         forward_count = 0
                                         hand_forward = True

                                    
                                else:
                                    forward_count = 0


                                 #If the hand position is 0.12 m more in y than the reference position for 5 consecutive readings, the hand has moved back
		                if(hand_position[1] > init_position[1] and abs(hand_position[1] - init_position[1]) > 0.12): #must change the axis probably. THis with respect to initial position important, like for the head tracking last year. FOr speed this doesnt work because we want to know which is the speed at the given moment. AS WE MOVE FORWARD IT BECOMES MORE NEGATIVE
                                    back_count +=1
                                    if(back_count == 5):
 
		                   	 print "Hand moved back" 
                                         write_hand("Hand moved back")
                                         back_count = 0
                                         hand_back = True

                                    
                                else:
                                    back_count = 0



                                 #If the hand position is 0.009 m more in z than the reference position for 5 consecutive readings, the hand has moved up
                                if(hand_position[2] > init_position[2] and abs(hand_position[2] - init_position[2]) > 0.009):  

                                    up_count +=1
                                    if(up_count == 5):
         
		                   	 print "Hand moved up" 
                                         write_hand("Hand moved up")
                                         gesture_h("Hand moved up")
                                         up_count = 0
                                         hand_up = True
                                else:
                                         up_count = 0
                               #If the hand position is 0.009 m less in z than the reference position for 5 consecutive readings, the hand has moved dow 
                                if(hand_position[2] < init_position[2] and abs(hand_position[2] - init_position[2]) > 0.009):
                                    down_count +=1
                                    if(down_count == 5):
         
		                   	 print "Hand moved down" 
                                         write_hand("Hand moved down")
                                         gesture_h("Hand moved down")
                                         down_count = 0
                                         hand_down = True
                                else:
                                         down_count = 0



                                #If the maximum velocity is higher than 0.2 and hand was not moving at such speed before (person had not started moving), determine they have begun to reach
                                #for th eobject
		                if(max(velocity)> 0.2 and movement_init == False): 


                                     move_start +=1
                                     if(move_start == 2):
				             print("person started moving")
                                             write_hand("Person started moving")
                                             gesture_h("Person started moving")
				             movement_init = True
		                           
				             init_time = datetime.now()
                                             move_start = 0
                                else:
                                            move_start = 0
                                #If the velocity of the hand has decrased after reaching a peak, it indicates hand is close to the object
		                if(max(velocity) < 0.1 and movement_init == True): 
                                     near_object += 1
                                     if(near_object == 2):
                 
				             print("nearing object")
                                             near_object = 0
                                else:
                                      near_object = 0

                      
                        #As long as the hand has been stabilised, start saving the hand coordinates and velocities to the rosbag               
                        if(init_position):
				write_hand("Hand position " + str(hand_position))
				position(hand_position)
				velocity_h(velocity)
				write_hand("Hand speed " + str(velocity))
				print(velocity)
	        prev_hand_position = hand_position





#Read PoseStamped message with hand coordinates
def skeleton(msg):
    global hand_position
    hand_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    

def hand_tracking():

    
    first_aruco = False
    hand_sub = rospy.Subscriber("/hand", PoseStamped, skeleton)
    initial_pose = []
    initial_or = []
    marker = False
    marker_position = []
    prev_marker_position = []
    moved = False
    time = 0.5
    prev_speed = []
    first_aruco = False
    goal_set = False
    height_from_table = 0
    motion = False
    cout_aruco_move = 0
    velocity_marker = 0
    global hand_up
    global hand_forward
    global grasped
    while not rospy.is_shutdown():
                    rospy.sleep(0.1)
                    hand_process()
		                                      
		                






if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and saubscribe over ROS.
        rospy.init_node('hand_tracking')
        result = hand_tracking()
        print("Moved")
    except rospy.ROSInterruptException:
	print("program interrupted before completion")
