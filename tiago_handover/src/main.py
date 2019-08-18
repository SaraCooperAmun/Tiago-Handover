#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sammy Pfeiffer

import rospy
import rosbag
from std_msgs.msg import Int32, String

    
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from nav_msgs.msg import Path 
from datetime import datetime
import conversions

import actionlib
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import tf
from actionlib import SimpleActionClient
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance, JointTrajectoryControllerState
from actionlib import SimpleActionClient, GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionFeedback
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_detection_msgs.msg import FaceDetections
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from moveit_msgs.msg import MoveGroupActionFeedback
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import yaml
import io
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

class Handover(object):
    def __init__(self):
        self.patient_id = '2019001' #Include ID of the participant
        self.my_rosbag = rosbag.Bag('/home/pal/sara_ws/patient_data/patientriggers'+self.patient_id+'block1.bag', 'w') #change for block

        self.grasp_robot = [-0.09, -0.1, -0.11]
        self.grasp_human = [-0.11, -0.12, -0.13] 

        self.cout_grasp_robot = 0
        self.cout_grasp_human = 0
        self.last_msg = None
        self.start_ac = None
        self.gripper_joints = None
        self.plan_compute = False
        self.moveit_status = 0
        self.torso_joints = []
        self.pos = []
        self.person = False
        self.arm_joints= None
        self.action = 'None'
        self.first_look = False
        self.plan_feedback = None
        self.marker_xtion = []
        self.time_object = 

        self.force_torque_sub = rospy.Subscriber('/wrist_ft',
                                                 WrenchStamped,
                                                 self.force_torque_cb,
                                                 queue_size=1)

        self.pub = rospy.Publisher('/trigger', String, queue_size=1)
        self.plan_done= rospy.Subscriber('/done', String, self.plan_state)

        
	self.scene = moveit_commander.PlanningSceneInterface()
	self.robot = moveit_commander.RobotCommander()

        self.group = moveit_commander.MoveGroupCommander("arm")
	self.eef_link=self.group.get_end_effector_link()
        self.scene.remove_attached_object(self.eef_link, name="object")
        self.scene.remove_world_object("object")
        		
        self.scene.remove_world_object("table")
	self.torso = SimpleActionClient(
            '/safe_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

	self.arm_torso = SimpleActionClient(
            '/safe_arm_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.cam_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo, timeout=None)


        self.client = SimpleActionClient('/safe_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_subscribe= rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, self.arm_state)
        self.face = rospy.Subscriber("/pal_face/faces", FaceDetections, self.callback_face) 
        self.client.wait_for_server()
        self.torso_subscribe= rospy.Subscriber('/torso_controller/state', JointTrajectoryControllerState, self.torso_state)
        self.action_begin = rospy.Subscriber('/action', String, self.action_feedback)
        self.action_begin_pub = rospy.Publisher('/action', String, queue_size = 1)
        self.aruco_tf = rospy.Subscriber('/xtion_aruco', PoseStamped, self.aruco_xtion_tf)
        self.send_waypoints = rospy.Publisher('/my_waypoints', Path, queue_size = 1)

	self.gripper_cmd = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)
	self.marker_position = []
        self.prev_marker_position = []
           
	self.head_cmd = rospy.Publisher(
				'/head_controller/command', JointTrajectory, queue_size=1)
	self.arm_controller = rospy.Publisher(
				'/arm_controller/command', JointTrajectory, queue_size=1)
        self.message = " "

	self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)   #specify the motion using play_motion
	if not self.play_m_as.wait_for_server(rospy.Duration(20)):
		rospy.logerr("Could not connect to /play_motion AS")
		exit()
        self.error = rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.moveit_state)
	rospy.loginfo("Connected!")
	self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path_josh', moveit_msgs.msg.DisplayTrajectory)
        self.say_client = SimpleActionClient('/tts', TtsAction)
	self.say_client.wait_for_server()
        self.ac = SimpleActionClient(
            '/head_controller/point_head_action', PointHeadAction)
        self.hand_sub = rospy.Subscriber("/hand", PoseStamped, self.skeleton)
        self.plan_done= rospy.Publisher('/plan_request', String, queue_size = 1)
        self.plan_feedback = rospy.Subscriber("/plan_done", String, self.plan_cb)
        self.skip_intro = True

        self.release_time = 0


	rate = rospy.Rate(1)
	self.listener = tf.TransformListener()
	
	rospy.loginfo("Looked down")

	self.hand_position = []
	self.prev_hand_position = []
	self.velocity = []
	self.time_hand = 0.1
	self.movement_init = False
	self.init_time = 0
	self.finish_time = 0
	self.init_position = []
	self.counter = 0
	self.track_time = 0
	self.up_count = 0
	self.forward_count = 0
	self.back_count = 0
	self.down_count = 0
	self.object_grasped = 0
	self.near_object = 0
	self.move_start = 0
        self.initial_pose = []
        self.initial_or = []
	self.hand_up = False
	self.hand_down = False
	self.hand_back = False
	self.hand_forward = False
        self.environment_set = False
	self.grasped = False
        self.rest_pose = []

        self.first = True
        self.initial_torque = []
        self.torque_after_grasp = []

        self.object_weight_x = 0
        #Dimensions of the objects used

        self.object = [0.25, 0.08, 0.04] 

        self.fl0 = 0
        self.m = 0 
        self.fzlg = 0.2 
        self.fg0 = 0
        self.fl = 0
        self.threshold = 0
        self.cout_actions = 0
        self.force_torque_sub = rospy.Subscriber('/wrist_ft',
                                                 WrenchStamped,
                                                 self.force_torque_cb,
                                                 queue_size=1)
        self.gripper_state = rospy.Subscriber('/gripper_controller/state', JointTrajectoryControllerState, self.gripper_state, queue_size=1)
        self.gripper_cmd = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)




        self.run()

    def force_torque_cb(self, msg):

        self.last_msg = msg

    def plan_cb(self, msg):

        self.plan_feedback = msg.data

 
    def write_txt(self, data): #Write logs to txt file
       
          file = open('/home/pal/sara_ws/patient_data/patientdata' + self.patient_id + "block1.txt", "a")
          date = datetime.today()
        
          file.write(str(data) + "    " + str(date) +"\n")



    def wrist_write(self):#Write wrist rosbag data to rosbag
       


          self.my_rosbag.write('wrist_ft', self.last_msg)



    def write(self, data): #Write string data to rosbag
       


                s = String()
                s.data =  str(data)
   
                self.my_rosbag.write('chatter', s) 
                self.write_txt(data)
     

    def aruco_xtion_tf(self, msg): #Update the object position with respect to the xtion_rgb_optical_frame to enable object tracking


       self.marker_xtion = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

      

    def action_feedback(self, msg): #Get feedback of button release
	  
	     self.action = msg.data
             release = 0
             print(self.action)
             if (self.action == "released"):
                    self.write(self.action)
                    self.release_time = datetime.now()
                    print("write person released")

    #Evaluates stability of the object, the speed based on the aruco marker, to determine if it is still enough and stable, or not             
    def object_stable(self):

           time = 0.1
           try:
               (trans,rot) = self.listener.lookupTransform('/base_footprint', '/aruco_marker_frame', rospy.Time(0))
                self.prev_marker_position = [trans[0], trans[1], trans[2]]
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                      print("exception")
				      rospy.sleep(0.1)
           rospy.sleep(time)
           try:
                                    
                                     (trans,rot) = self.listener.lookupTransform('/base_footprint', '/aruco_marker_frame', rospy.Time(0))
                                      self.marker_position = [trans[0], trans[1], trans[2]] 
                                     if(self.prev_marker_position):
			       
					     self.velocity_marker = [abs(self.marker_position[0] - self.prev_marker_position[0])/time, abs(self.marker_position[1] - self.marker_position[1])/time, abs(self.marker_position[2] - self.prev_marker_position[2])/time] 
                                             if(self.velocity_marker[2] < 0.01)
                                                     return True
                                             else:
                                                      return False
                                     
                                     self.prev_marker_position = [trans[0], trans[1], trans[2]] 
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                      print("exception")
				      rospy.sleep(0.1)
				      
    def plan_state(self, msg):
	  
	     self.plan_computed= msg.data

    def gripper_state(self, msg):
        
              self.gripper_joints = msg.actual.positions

    def arm_state(self,msg):
        
              self.arm_joints = msg

    #Hand tracking with Astra Orbbec and Nuitrack software. Not used in the user-study
    def hand_process(self):

        if(self.hand_position):

                if(self.prev_hand_position):
                        #Compute velocity of the hand by difference in position in a time-frame of 0.1 s

                	self.velocity = [abs(self.hand_position[0] - self.prev_hand_position[0])/self.time_hand, abs(self.hand_position[1] - self.prev_hand_position[1])/self.time_hand, abs(self.hand_position[2] - self.prev_hand_position[2])/self.time_hand]
                      
                        #Determine if hand is at reference position (button press) and stable. It must be still (velocity less than 0.01 m/s) for 5 consecutive readings)

                        if(len(self.init_position)==0):
                               
		                if(self.counter < 5):
		                      if(max(self.velocity) < 0.01):
		                           self.counter+=1
		                      else:
		                           self.counter = 0
                                      print("not stabilised")
		                else:
                                     #Save the initial hand position to be taken as reference
		                     self.init_position = [self.hand_position[0], self.hand_position[1], self.hand_position[2]]
		                     
		                     print(self.init_position, "hand stabilised")

                        else:
                
                                #Once the hand is stable, start detecting gestures.  Taking into account the reference system for the Astra camera:
                                         # hand moved forward: decrease in y
                                         # hand moved back: increase in y
                                         # hand moved up: increase in z
                                         # hand moved down: decrease in z

                                #If the hand position is 0.12 m less in y than the reference position for 5 consecutive readings, the hand has moved forward
                                
		                if(self.hand_position[1] < self.init_position[1] and abs(self.hand_position[1] - self.init_position[1]) > 0.12):  
                                    self.forward_count +=1
                                    if(self.forward_count == 5):
 
		                   	 print "Hand moved forward"
		                   	 self.forward_count = 0
                                         self.hand_forward = True
                                    
                                else:
                                    self.forward_count = 0

                              #If the hand position is 0.12 m more in y than the reference position for 5 consecutive readings, the hand has moved back
		                if(self.hand_position[1] > self.init_position[1] and abs(self.hand_position[1] - self.init_position[1]) > 0.12): #must change the axis probably. THis with respect to initial position important, like for the head tracking last year. FOr speed this doesnt work because we want to know which is the speed at the given moment. AS WE MOVE FORWARD IT BECOMES MORE NEGATIVE
                                    self.back_count +=1
                                    if(self.back_count == 5):
 
		                   	 print "Hand moved back" 
                                         self.back_count = 0
                                         self.hand_back = True

                                    
                                else:
                                    self.back_count = 0

                               #If the hand position is 0.009 m more in z than the reference position for 5 consecutive readings, the hand has moved up

                                if(self.hand_position[2] > self.init_position[2] and abs(self.hand_position[2] - self.init_position[2]) > 0.009): 
                                    self.up_count +=1
                                    if(self.up_count == 5):
         
		                   	 print "Hand moved up" 
                                         self.up_count = 0
                                         self.hand_up = True
                                else:
                                         self.up_count = 0

                               #If the hand position is 0.009 m less in z than the reference position for 5 consecutive readings, the hand has moved dow 
                                if(self.hand_position[2] < self.init_position[2] and abs(self.hand_position[2] - self.init_position[2]) > 0.009):
                                    self.down_count +=1
                                    if(self.down_count == 5):
         
		                   	 print "Hand moved down" 

                                         self.down_count = 0
                                         self.hand_down = True
                                else:
                                         self.down_count = 0
                                #If the maximum velocity is higher than 0.2 and hand was not moving at such speed before (person had not started moving), determine they have begun to reach
                                #for th eobject
		                if(max(self.velocity)> 0.2 and self.movement_init == False): 
                                     self.move_start +=1
                                     if(self.move_start == 2):
				             print("person started moving")
				             self.movement_init = True
		                           
				             self.init_time = datetime.datetime.now()
                                             self.move_start = 0
                                else:
                                            self.move_start = 0
                                #If the velocity of the hand has decrased after reaching a peak, it indicates hand is close to the object

		                if(max(self.velocity) < 0.1 and self.movement_init == True):
                                    self.near_object += 1
                                     if(self.near_object == 2):
                 
				             print("nearing object")
                                             self.near_object = 0
                                else:
                                      self.near_object = 0

                                            


                self.prev_hand_position = self.hand_position

    def skeleton(self,msg): 
	    
	    self.hand_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def callback_face(self, data): #Callback for face detected. Return self.person = true if a person has been detected
           try:
              self.pos = [data.faces[0].x, data.faces[0].y]
         
              self.person = True
           except:
              rospy.sleep(0.01)
              self.person = False 

    def torso_state(self, msg):
           self.torso_joints = msg


    def moveit_state(self, msg):
	    
	     self.moveit_status = msg.status.status     
         

    def say_something(self, text):

		    # Create a goal to say our sentence
		    goal = TtsGoal()
		    goal.rawtext.text = text
		    goal.rawtext.lang_id = "en_GB"
		    # Send the goal and wait
		    self.say_client.send_goal_and_wait(goal)

    #Look at human face or aruco marker
    def look_at(self, x, y, block=True): 

        g = PointHeadGoal() #Create a PointHeadGoal message
        g.pointing_frame = 'xtion_rgb_optical_frame'
        g.pointing_axis.z = 1.0

        error_x = abs(x)
        error_y = abs(y)
        error = max(error_x, error_y) #compute maximum error in vertical and horizontal height with respect to the image center
        kp= 0.3 #set the gain for the controller. The higher it is the slower and smoother movement
        g.max_velocity = 1.0
        g.min_duration = rospy.Duration(float(kp/error)) #Adjust the motion duration with respect to the error. The bigger error, the quicker
        g.target.header.frame_id = 'xtion_rgb_optical_frame'
        g.target.point.x = x
        g.target.point.y = y
        g.target.point.z = 1.0
       
        if block:
            self.ac.send_goal_and_wait(g)
        else:
            self.ac.send_goal(g)


    #Adjust torso based on person's height. It reads the coordinates of the tracked face (x,y) and if Tiago is lower than the face's height, it raises the torso. Otherwise, it lowers, proportinally to the distance.
    def torso_adjust_adaptive(self, y, torso):

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names= ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        print(torso, y)
        if(y< 0): 
           height = torso[0]+abs(y) 
        elif(y> 0):
           height = torso[0] - abs(y)
        if(height < 0.2):
               height = 0.2
        jtp.positions = [height]
       
        error = abs(y) 
        kp = 0.2 
	jtp.velocities = [0.1]
        
	jtp.time_from_start = rospy.Duration(float(kp/error)) #adjust speed of motion depending on error
	goal.trajectory.points.append(jtp)
        print(goal)
	self.torso.send_goal(goal)
        self.torso.wait_for_result() 

    #Return to resting position
    def begin_position(self, mymotion):

	    data = rospy.get_param("mymotions")

            points= data.get('play_motion').get('motions').get(mymotion).get('points')[0]
         

	    goal = FollowJointTrajectoryGoal()
	    goal.trajectory.joint_names= ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']


            jtp = JointTrajectoryPoint()
		         
	    jtp.positions = points.get('positions')

	    jtp.velocities = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]
	    jtp.time_from_start = rospy.Duration(2.0) 
	    goal.trajectory.points.append(jtp)

	    self.client.send_goal(goal)

	    self.client.wait_for_result()
            rospy.loginfo("Waiting for result...")
	    action_ok = self.client.wait_for_result(rospy.Duration(30.0))

	    state = self.client.get_state()         



    #Check alignment in y (horizontal) between end-effector and the object
    def check_alignment_y(self):

                        trans_gripper = [self.group.get_current_pose().pose.position.x,self.group.get_current_pose().pose.position.y,self.group.get_current_pose().pose.position.z] 
			marker_2 = False
                        while (not marker_2):
				try:
	
				    (trans,rot) = self.listener.lookupTransform('/base_footprint', '/aruco_marker_frame', rospy.Time(0))

                                    marker_2 = True

				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				   
				   continue

                        print("Trans between gripper and object", trans_gripper, trans)
                        if (abs(trans_gripper[1] - trans[1]) < 0.1): 
                             return True #Gripper aligned, proceed to grasping
                        else: 
                             return False#Gripper not aligned, replan

    #Check alignment in z (depth) between end-effector and the object
    def check_alignment_z(self):

                        trans_gripper = [self.group.get_current_pose().pose.position.x,self.group.get_current_pose().pose.position.y,self.group.get_current_pose().pose.position.z] 
			marker_2 = False
                        while (not marker_2):
				try:
		
				    (trans,rot) = self.listener.lookupTransform('/base_footprint', '/aruco_marker_frame', rospy.Time(0))

                                    marker_2 = True

				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				   
				   continue

                        if (abs(trans_gripper[2] - trans[2]) < 0.2):
                             return True #Gripper aligned, grasp object
                        else: 
                             return False#Gripper not aligned, replan



    def run(self):

        marker = False
       
        self.message ="None"
        self.action_begin_pub.publish(self.message)
        
        if (self.skip_intro == False):   #Set to True if we want to skip the introduction
	        self.scene.remove_world_object("table")
		self.scene.remove_world_object("right_restrict") 
		self.scene.remove_world_object("left_restrict") 
		self.scene.remove_world_object("top_restrict")              
		goal = PlayMotionGoal()
		goal.motion_name = 'pregrasp03'
		goal.skip_planning = False
		self.play_m_as.send_goal_and_wait(goal)
                rospy.sleep(0.4)
                print(self.moveit_status)
                if (self.moveit_status != 3):
                   	self.play_m_as.send_goal_and_wait(goal)
              
		jt = JointTrajectory() 
		jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		jtp = JointTrajectoryPoint()
		jtp.positions = [0, 0]
		jtp.time_from_start = rospy.Duration(2.0) 
		jt.points.append(jtp)
		self.head_cmd.publish(jt)
        
		goal = PlayMotionGoal() 
		goal.motion_name = 'hello' 
		goal.skip_planning = False
		self.play_m_as.send_goal(goal)
		self.say_something("Hello, I am Tiago.I look forward to doing this experiment with you")
                rospy.sleep(2)
		img_proc = PinholeCameraModel()
		img_proc.fromCameraInfo(self.cam_info)

		while (not self.first_look):
		   #Detect face and adjust torso based on participant's height
		   if(self.person):
		        print("person detected")
		        coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1])) 
                        torso_adapt = self.torso_adjust_adaptive(coordinate[1], list(self.torso_joints.desired.positions))
		        self.first_look = True
		        rospy.sleep(5)

			waypoints = []
			wpose = self.group.get_current_pose().pose
			wpose.position.x = 0.613
			wpose.position.y = -0.2
			wpose.position.z = 0.864
			wpose.orientation.x = -0.567
			wpose.orientation.y = 0.11
			wpose.orientation.z = -0.08
			wpose.orientation.w = 0.81
			waypoints.append(copy.deepcopy(wpose))
	                #Set the arm's resting position 
			(plan, fraction) = self.group.compute_cartesian_path(
								       waypoints,   # waypoints to follow
								       0.02,        # eef_step
								       0.0,True)         # jump_threshold

                        self.group.execute(plan, wait=True)
                        self.rest_pose = list(self.arm_joints.desired.positions)
                        for i in range(len(self.arm_joints.desired.positions)):
                        	self.rest_pose.append(list(self.arm_joints.desired.positions)[i]) 

		

        
        
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
 
            self.process_cues()
            r.sleep()

    def process_cues(self):

        self.scene.remove_attached_object(self.eef_link, name="object")
        self.scene.remove_world_object("object")
  
   
        
	while (not self.environment_set):
                        #Prepare the collision environment with respect to the marker
			self.scene.remove_world_object("table")
			self.scene.remove_world_object("right_restrict") 
			self.scene.remove_world_object("left_restrict") 
			self.scene.remove_world_object("top_restrict") 
			try:

						  jt = JointTrajectory() 
						  jt.joint_names = ['head_1_joint', 'head_2_joint'] 
						  jtp = JointTrajectoryPoint()
						  jtp.positions = [0, -0.26]  looks before
						  jtp.time_from_start = rospy.Duration(1.0)  head
						  jt.points.append(jtp)
						  self.head_cmd.publish(jt) #Look up to see the marker
		                                  rospy.sleep(1)
				
						  (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_goal', rospy.Time(0))
				                  #Collision object: table
						  object_pose = geometry_msgs.msg.PoseStamped()

						  object_pose.header.frame_id = "aruco_marker_frame"
						  object_pose.pose.position.x=-self.object[0] - 0.05
						  object_pose.pose.position.y=-0.7 
						  object_pose.pose.orientation.w=1.0
						  object_name = "table"
						  self.scene.add_box(object_name, object_pose, size=(0.1,2,2))
  				                  #Collision object: left block
						  obstacle_pose = geometry_msgs.msg.PoseStamped()
						  obstacle_pose.header.frame_id = "aruco_marker_frame"
						  obstacle_pose.pose.position.y=-0.5
						  obstacle_pose.pose.position.z = 1.7 

						  obstacle_pose.pose.orientation.w=1.0
						  object_name = "left_restrict"
						  self.scene.add_box(object_name, obstacle_pose, size=(2,2,2))

				                  #Collision object: top block

						  obstacle_pose = geometry_msgs.msg.PoseStamped()
						  obstacle_pose.header.frame_id = "aruco_marker_frame"

						  obstacle_pose.pose.position.x=1.5 
		
						  obstacle_pose.pose.orientation.w=1.0
						  object_name = "top_restrict"
						  self.scene.add_box(object_name, obstacle_pose, size=(2,2,2))
				                  
                                                  #Collision object: right block


						  obstacle_pose = geometry_msgs.msg.PoseStamped()
						  obstacle_pose.header.frame_id = "aruco_marker_frame"

						  obstacle_pose.pose.position.y=-0.5
						  obstacle_pose.pose.position.z = -1.2

						  obstacle_pose.pose.orientation.w=1.0
						  object_name = "right_restrict"
						  self.scene.add_box(object_name, obstacle_pose, size=(2,2,2))

				                  #Collision object: object
						  object_pose = geometry_msgs.msg.PoseStamped()
						  object_pose.header.frame_id = "aruco_marker_frame"

						  object_pose.pose.position.x= -0.1
						  object_pose.pose.orientation.w=1.0
						  object_name = "object"
						  self.scene.remove_attached_object(self.eef_link, name="object")
						  self.scene.remove_world_object("object")

                                                  #Set initial object position
						  (trans2,rot2) = self.listener.lookupTransform('/base_footprint', '/place_goal', rospy.Time(0))
				                  self.initial_pose = [trans2[0], trans2[1], trans2[2]]  
		  				  self.initial_or = [rot2[0], rot2[1], rot2[2], rot2[3]]
                
                                                  print ("initial pose of object",trans2, rot2)
						  goal = PlayMotionGoal()
						  goal.motion_name = 'open_gripper'
						  goal.skip_planning = False
						  self.play_m_as.send_goal_and_wait(goal)
		                                  self.environment_set = True
		                                  print("ENVIRONMENT READY")

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					      continue	



	jt = JointTrajectory() 
	jt.joint_names = ['head_1_joint', 'head_2_joint'] 
	jtp = JointTrajectoryPoint()
	jtp.positions = [0, -0.75] 
	jtp.time_from_start = rospy.Duration(1.0)
	jt.points.append(jtp)
	self.head_cmd.publish(jt)
        marker = False

        self.message ='not moving'
        print(self.action, self.message)
	marker_position = []
	prev_marker_position = []
        initial_pose = []
        initial_or = []
        initial_gaze = []
	moved = False
    
        cout_aruco_move = 0
        velocity_marker = 0
       
	time = 0.4 
        time_human = 0.1
	prev_speed = []
	first_aruco = False
	goal_set = False
        self.message =" "

	goal = PlayMotionGoal()

	goal.skip_planning = False

            

        if(self.action!= 'None'):
             self.cout_actions += 1
             self.write(self.action)

             print(self.cout_actions)


             if(self.action == 'finished'): #Indicates experiment has ended
		   goal = PlayMotionGoal()
		   goal.motion_name = 'pregrasp02only'
		   goal.skip_planning = False
		   self.play_m_as.send_goal_and_wait(goal)
                   rospy.sleep(2)

		   jt = JointTrajectory()
		   jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		   jtp = JointTrajectoryPoint()
		   jtp.positions = [0, 0]
		   jtp.time_from_start = rospy.Duration(1.5) 
		   jt.points.append(jtp)
		       
		   self.head_cmd.publish(jt)
		
                   self.say_something("We have finished the experiment! Thank you for taking part!")
	
                   self.message ="None"
                   self.action_begin_pub.publish(self.message)


             if(self.action == 'blockdone'): #Executed when a block has ended

		   goal = PlayMotionGoal()
		   goal.motion_name = 'pregrasp02only'
		   goal.skip_planning = False
		   self.play_m_as.send_goal_and_wait(goal)
                   rospy.sleep(2)

		   jt = JointTrajectory() 
		   jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		   jtp = JointTrajectoryPoint()
		   jtp.positions = [0, 0] 
		   jtp.time_from_start = rospy.Duration(1.5)
		   jt.points.append(jtp)
		       
		   self.head_cmd.publish(jt)
		
                   self.say_something("We have finished a block, you can take a rest if you like")
	
                   self.message ="None"
                   self.action_begin_pub.publish(self.message)
                   self.my_rosbag.close()
               
                    
	     #Robot-to-human handover
             if(self.action == 'robot'):
                rospy.sleep(1)
	        jt = JointTrajectory() 
	        jt.joint_names = ['head_1_joint', 'head_2_joint'] 
	        jtp = JointTrajectoryPoint()
	        jtp.positions = [0, -0.56] 
	        jtp.time_from_start = rospy.Duration(1.0) 
	        jt.points.append(jtp)
               
	        self.head_cmd.publish(jt)
                rospy.sleep(1.0)
                touch_links=['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_grasping_frame']

                #Detect marker

		while(not marker):
		        print("Action started") 
			try:
				  marker = True 
                                  #Lookup transform between the base_footprint and arm_goal, which specifies the pregrasp position
				  (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_goal', rospy.Time(0))
                                  print("waiting")
                                  self.write("Marker location " + str(trans))
                                  self.write("Marker orientation " + str(rot))

                                  #Add object as collision object
				  object_pose = geometry_msgs.msg.PoseStamped()
				  object_pose.header.frame_id = "aruco_marker_frame"

				  object_pose.pose.position.x= -0.1
				  object_pose.pose.orientation.w=1.0
				  object_name = "object"
		                  self.scene.remove_attached_object(self.eef_link, name=object_name)
				  self.scene.remove_world_object(object_name)

                              
				  self.scene.add_box(object_name, object_pose, size=(self.object[0], self.object[1], self.object[2]))
				  goal = PlayMotionGoal()
				  goal.motion_name = 'open_gripper'
				  goal.skip_planning = False
				  self.play_m_as.send_goal_and_wait(goal)
		                  x = self.marker_xtion[0]
                                  y = self.marker_xtion[1]
                                  #Track object          
	                      	  self.look_at(x,y, block=False) 
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			      continue
                        grasped_object = 0 

                        #Test the 3 grasp points. If none of them ensure the robot grasps the object report failure
                        while (grasped_object == 0 and self.cout_grasp_robot < 3):
		                send_data = Path()

				waypoints = []
				wpose = self.group.get_current_pose().pose
				wpose.position.x = trans[0] 
				wpose.position.y = trans[1]
				wpose.position.z = trans[2]
				wpose.orientation.x = rot[0]
				wpose.orientation.y = rot[1]
				wpose.orientation.z = rot[2]
				wpose.orientation.w = rot[3]
		                
		                new_pose = PoseStamped()
		                new_pose.pose = wpose
		                send_data.poses.append(copy.deepcopy(new_pose))
				wpose.position.x +=  0.12 
				waypoints.append(copy.deepcopy(wpose))
		                new_pose = PoseStamped()
		                new_pose.pose = wpose
		                send_data.poses.append(copy.deepcopy(new_pose))
		                self.message="moving"

				self.pub.publish(self.message)
		                self.write("Starts moving")
		                init_movement = datetime.now()
		                while (self.plan_feedback == "Done"):
		                      rospy.sleep(0.05)
		                msg = String()
		                msg.data = "request sent"
		                self.plan_done.publish(msg)
		           
		                self.send_waypoints.publish(send_data)
		                print(self.plan_feedback)

		                while (self.plan_feedback!= "Done"):


				       x, y = self.marker_xtion[0], self.marker_xtion[1]   #track object as it picks it
				       if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
								    self.look_at_(x,y, block=False)
						                    rospy.sleep(0.01)
	  
		                      rospy.sleep(0.05)



		                print("plan computed")
		                msg = String()
		                msg.data = ""
		                self.plan_done.publish(msg)
		               

		                print(self.moveit_status)
		                response = self.check_alignment_y()
		                response_z = self.check_alignment_z() 
		                rospy.sleep(0.3)
                                #If the gripper is not alignment with the object or there is a MoveIt error, try the nxt grasp point
		                if(self.moveit_status == 4 or response == False or response_z == False ):

                                                self.cout_grasp_robot += 1
                                               
                                                trans[0] = self.grasp_robot[self.cout_grasp_robot]
                                else:
 

		                     grasped_object = 1

                         if (self.cout_grasp_robot == 3 and grasped_object == 0): #If after 3 tries it does not manage to grasp, return to initial position
   						print("Failure")
						goal = PlayMotionGoal()
						goal.motion_name = 'grasp17' 
						goal.skip_planning = False
						self.play_m_as.send_goal_and_wait(goal)

		                                self.message ="Unsucessfull"  
                                                break
                        grasped_object = 0
                        self.cout_grasp_robot = 0         
                        #Read initial gripper force values
                        f = self.last_msg.wrench.force
                        if (True):
                                self.initial_torque = [f.x, f.y, f.z]
                                self.write("Initial force " + str(self.initial_torque))
				jt = JointTrajectory()
				jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']
				jtp = JointTrajectoryPoint()
				   
				jtp.positions = [0.025, 0.025] #close gripper
				jtp.time_from_start = rospy.Duration(0.5) 
				jt.points.append(jtp)
				self.gripper_cmd.publish(jt)
				rospy.loginfo("Closed")
                        #Move end-effector to the handover location, 0.12 m above the current position
                        send_data = Path()

			waypoints = []
			wpose = self.group.get_current_pose().pose

                        wpose.position.z +=  0.12
                        new_pose = PoseStamped()
                        new_pose.pose = wpose
                        send_data.poses.append(copy.deepcopy(new_pose))
                        
			waypoints.append(copy.deepcopy(wpose))
 
                        while (self.plan_feedback == "Done"):
                              rospy.sleep(0.05)

                        
                        msg = String()
                        msg.data = "request sent"
                        self.plan_done.publish(msg)
                   
                        self.send_waypoints.publish(send_data)
                        print(self.plan_feedback)

               
                        rospy.sleep(1)
		        jt = JointTrajectory() #this is to specify the play motion. 
			jt.joint_names = ['head_1_joint', 'head_2_joint'] #what joints are responsible for head control 
			jtp = JointTrajectoryPoint()
			jtp.positions = [0, -0.1] #change the coordinates of the head so that robot looks before
			jtp.time_from_start = rospy.Duration(0.6) #how fast the robot should move the head
			jt.points.append(jtp)
                        init_action = datetime.now()
                        while (self.plan_feedback!= "Done"):


                               duration = datetime.now() - init_action
                               if(duration.total_seconds() > 0.4): #0.5 is the average time needed by Tiago to raise the object
                                   x, y = self.marker_xtion[0], self.marker_xtion[1]   #track object while the robot is raising it. Show gaze pattern: object/handover location, human face, back to object
			           if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
								    self.look_at(x,y-0.02, block=False)
						                    rospy.sleep(0.01)
                                   #Look back at human face
                                   if(self.person): 
				       	img_proc = PinholeCameraModel()
					img_proc.fromCameraInfo(self.cam_info)
					coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
					if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
							    self.look_at(coordinate[0],coordinate[1], block=False)
                                        rospy.sleep(0.1)

                          

			       x, y = self.marker_xtion[0], self.marker_xtion[1]   
                               #Look back at object
			       if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
								    self.look_at(x,y, block=False)
						                    rospy.sleep(0.01)


		              
				         
                               rospy.sleep(0.05)


                        print("plan computed")


                        rospy.sleep(0.3) #so that tiago looks before                        
			self.head_cmd.publish(jt)

                        msg = String()
                        msg.data = ""
                        self.plan_done.publish(msg)
		

                     	f = self.last_msg.wrench.force
                        print(self.initial_torque[0], f.x)
                        self.wrist_write()
                        rospy.sleep(0.2)
                        if(f.x > (self.initial_torque[0] + 0.5)): #make sure object has been grasped, otherwise we will retreat

			    f = self.last_msg.wrench.force
			    self.torque_after_grasp = [f.x, f.y, f.z]
                            self.write("Force after grasp " + str(self.torque_after_grasp))
         
			    self.object_weight_x = self.torque_after_grasp[0] - self.initial_torque[0] 
                            
			    print("Torque after grasping ", self.torque_after_grasp)
                            self.wrist_write()
			    self.fl0 = self.object_weight_x
			    self.fl = f.x #the load is the same as f.x, corresponds to weight of object + initial gripper load

			    self.fg0 =self.fl0 
			    self.m = (self.fg0 - self.fzlg)/self.fl0
			    self.threshold = self.initial_torque[0] - self.object_weight_x*0.02 #threshold should be around 2 or 10 percent of.
                            self.first = False
                            self.write("Object weight :" + str(self.object_weight_x))
                            self.write("FL0 : " + str(self.fl0))
                            self.write("FG0 : " + str(self.fg0))
                            self.write("m : " + str(self.m))
                            self.write("threshold: " + str(self.threshold))
                        else:
					    goal = PlayMotionGoal()
				            self.scene.remove_world_object("table")
				            self.scene.remove_world_object("right_restrict") 
					    self.scene.remove_world_object("left_restrict") 
					    self.scene.remove_world_object("top_restrict") 
					    goal.motion_name = "pregrasp02only" 
					    goal.skip_planning = False
					    self.play_m_as.send_goal_and_wait(goal)
                                  
				            print("error")
					   
				            self.message ="Unsucessfull"
					    self.pub.publish(self.message)
					    print self.message
					    rospy.sleep(0.5)
					    self.action = 'None'
                                            self.write(self.message)
                                            break
                          



                        self.write("Initiate object transfer")
                        init_grasping = datetime.now()
		        f = self.last_msg.wrench.force
		        counter = 0
                        gripper_open = False
                        start_grasping = 0
                        
		        while (not gripper_open): 
		                  rospy.sleep(0.05)
                                  #Look at human face as Tiago is offering object
                                  if(self.person):
                                     
				       	img_proc = PinholeCameraModel()
					img_proc.fromCameraInfo(self.cam_info)
					coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
					if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
							    self.look_at(coordinate[0],coordinate[1], block=False)

                                        rospy.sleep(0.1)
		                  f = self.last_msg.wrench.force
		                  self.head_cmd.publish(jt)
		                  counter += 1 #put a counter so as to put a maximum to wait for the human
		                  if(counter > 10): 
		                        print("Please take it") 
		                        counter = 0 

			          if (not self.first):
          

				    jt = JointTrajectory() #this is to specify the play motion. 
				    jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint'] 
                           
				    if(f.z > (self.initial_torque[2]+0.5)): #if someone is pulling
					 
                                         if(start_grasping == 0):
                                             self.write("Human starts taking object")
                                             if(self.action == "released"):
                                                   release_grasp_duration = datetime.now() - self.release_time
                                                   self.write("Human duration release to grasp : "+ str(release_grasp_duration))
                                                   self.action_begin_pub.publish("None")
                                                   self.release_time = 0

                                             start_grasping = 1
					 if(f.x > self.threshold): 
					       jtp.positions = [list(self.gripper_joints)[0] + 0.001, list(self.gripper_joints)[0]  + 0.001]  
 

					 else:
					     jtp.positions = [0.03, 0.03] 

                                    if(f.x < self.threshold or (f.z  >= (self.initial_torque[2] +  1.5))):
                                               gripper_open = True
                                               jtp.positions = [0.03, 0.03]


                                    object_stability = self.object_stable() #Check stability of the object. If velocity of the object is higher than 0.1 it indicates the object may have risk of falling, and Tiago regrasps
                                    if not object_stability:
                                        jtp.positions = [0.025, 0.025] 
                                    
				    jtp.time_from_start = rospy.Duration(0.2) 
				    jt.points.append(jtp)
 
				    self.gripper_cmd.publish(jt)
                                    print("Gripper positions ", jtp.positions)
                                    print("Torque ", f, "Threshold ", f.x)

                                    self.write("Fx :" + str(f.x))
                                    self.write("Fy : " + str(f.y))
                                    self.write("Fz : " + str(f.z))
                                    self.write("Grip force : " + str(grip))
                                    self.write("Desired_force : " + str(desired_force))
                                    self.write("Grip positions: " + str(jtp.positions))

                                    self.wrist_write()
                                    rospy.sleep(0.2)

                        if(gripper_open):
                                            #Look at object when exchanging
					    x, y = self.marker_xtion[0], self.marker_xtion[1] 
					    if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
									    self.look_at(x,y, block=False)
								            rospy.sleep(0.01)
                                            self.write("Gripper opening")
                                            
                                            jt = JointTrajectory()
                                            jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint'] 
                                            jtp.positions = [0.04, 0.04]
                                            jtp.time_from_start = rospy.Duration(0.5)
                                            jt.points.append(jtp)
                                            self.gripper_cmd.publish(jt)
                                            rospy.sleep(0.5)
                                            self.write("Transfer finished")

                                            grasping_duration = datetime.now() - init_grasping
                                            self.write("Total object transfer duration " + str(grasping_duration))
                                            

				      	    self.scene.remove_attached_object(self.eef_link, name="object")
					    waypoints = []
                                            #Begin retreat
                                            send_data = Path()
					    wpose = self.group.get_current_pose().pose
					    wpose.position.x -= 0.16
				            wpose.position.z -= 0.14
					    waypoints.append(copy.deepcopy(wpose))
				            new_pose = PoseStamped()
				            new_pose.pose = wpose
					    send_data.poses.append(copy.deepcopy(new_pose))
						
					    waypoints.append(copy.deepcopy(wpose))
			 
					    while (self.plan_feedback == "Done"):
						      rospy.sleep(0.05)

						
					    msg = String()
					    msg.data = "request sent"
					    self.plan_done.publish(msg)
					   
					    self.send_waypoints.publish(send_data)
					    print(self.plan_feedback)

					    while (self.plan_feedback != "Done"):
					      x, y = self.marker_xtion[0], self.marker_xtion[1]   #track object as it is placed back
					      if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
										    self.look_at(x,y, block=False)
										    rospy.sleep(0.01)
				              rospy.sleep(0.05)

					    print("plan computed")

				            rospy.sleep(0.2)


					    self.scene.remove_world_object("object")


                                            self.begin_position('grasp17') #go to initial position
                                            robot_duration = datetime.now() - init_movement
					    if (self.message != "Unsuccesfull"):
				            	self.message ="Succesfull"
					    self.pub.publish(self.message)

					    self.action = 'None'
                                            self.message =" "
                                            self.write("Stop moving. Duration " + str(robot_duration))
                                            break
						       
		              
                                    
                marker = False


#Human-to-robot handover
             if(self.action == 'human'):


                       #Show gaze pattern: human face, look slightly down to detect object, and track object
                       rospy.sleep(1.3)

                       jt = JointTrajectory() 
		       jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		       jtp = JointTrajectoryPoint()
		       jtp.positions = [0, -0.2] 
		       jtp.time_from_start = rospy.Duration(0.6) 
		       jt.points.append(jtp)
		       self.head_cmd.publish(jt)
                       rospy.sleep(0.6)

                       #Look at human
		       if(self.person):
			       	img_proc = PinholeCameraModel()
				img_proc.fromCameraInfo(self.cam_info)
				coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
				if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
						    self.look_at(coordinate[0],coordinate[1], block=False)




		       jt = JointTrajectory() 
		       jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		       jtp = JointTrajectoryPoint()
		       jtp.positions = [0, -0.15] 
		       jtp.time_from_start = rospy.Duration(1.0) 
		       jt.points.append(jtp)
		       self.head_cmd.publish(jt)
                       #Look at object
		       x, y = self.marker_xtion[0], self.marker_xtion[1]
		       if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
							    self.look_at(x,y, block=False)
				                            rospy.sleep(0.01)

                       self.scene.remove_attached_object(self.eef_link, name="object")
		       self.scene.remove_world_object("object")
		       goal = PlayMotionGoal()
		       goal.motion_name = 'open_gripper'
		       goal.skip_planning = False
		       self.play_m_as.send_goal_and_wait(goal)
                       count_human = 0


                       rospy.sleep(0.6)

		       while (not goal_set or count_human < 20):
			   rospy.sleep(time)   
                           count_human += 1
                           print(count_human)
                           
                           
			   try:

                                     (trans,rot) = self.listener.lookupTransform('/base_footprint', '/human_goal', rospy.Time(0))

                                     self.object_pose(trans)
		                     
		                     if not(first_aruco):

                                          self.write("Waiting to take object")

		                          jt = JointTrajectory() 
					  jt.joint_names = ['head_1_joint', 'head_2_joint'] 
					  jtp = JointTrajectoryPoint()
		    			  jtp.positions = [0, -0.15]  before
					  jtp.time_from_start = rospy.Duration(0.5) 
					  jt.points.append(jtp)
							  
					  print("waiting to put object")
           
                              
                                          first_aruco = True


				     marker_position = [trans[0], trans[1], trans[2]] 
				     if(prev_marker_position):
			       
					     velocity_marker = [abs(marker_position[0] - prev_marker_position[0])/time, abs(marker_position[1] - marker_position[1])/time, abs(marker_position[2] - prev_marker_position[2])/time] 

					     if(velocity_marker[2] > 0.01 and not moved):
						     x = self.marker_xtion[0]
                                                     y = self.marker_xtion[1]
                                                     self.write("Human starts grasping")
                                                     print("Human starts grasping")
                                                     human_grasp_time = datetime.now()
                                                     print(self.action)
                                                     if(self.action == "released"):
		                                           release_grasp_duration = human_grasp_time - self.release_time
		                                           self.write("Human duration release to grasp : "+ str(release_grasp_duration))
		                                           self.action_begin_pub.publish("None")
		                                           self.release_time = 0
                                                           print("calculate release time")
                                                     else: 
                                                           release_grasp_duration = datetime.now()




					     if(velocity_marker[2] > 0.02):  
		                                   cout_aruco_move +=1
		                                   if(cout_aruco_move == 1):
						    	 moved = True
		                                         cout_aruco_move = 0
		                                   x = self.marker_xtion[0]
                                                   y = self.marker_xtion[1]
                                                   self.write("Object moving at speed " + str(velocity_marker[2]))
                                                   self.message="Object reached"
						   print self.message
						   self.pub.publish(self.message)
                                                   if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
	                      			      self.look_at(x,y, block=False)
		                             else: 
		                                   cout_aruco_move = 0
				       	           img_proc = PinholeCameraModel()
						   img_proc.fromCameraInfo(self.cam_info)
					    	   coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
                                                   if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
			    	          		self.look_at(coordinate[0], coordinate[1], block=False)
					     if(moved):



		                                      if(velocity_marker[2] < 0.01): 

							   moved = False
						           goal_set = True
                                                           count_human = 0
                                                           self.write("Object raised")
                                                           self.write("Velocity :" + str(velocity_marker[2]))
                                                           self.write("Position :" + str(trans))
                                                           rospy.sleep(0.5)
                                                           object_raise_time = datetime.now()
                                                           total_raise = object_raise_time - release_grasp_duration
                                                           self.write("Object raise time :"+ str(total_raise))
                                                         


                                                           init_grasping = datetime.now()
							   self.message="Object raised"
							   print self.message
							   self.pub.publish(self.message)

                       
		                                  	   x = self.marker_xtion[0]
                                                           y = self.marker_xtion[1]
                                                           if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
	                     				        self.look_at(x,y, block=False)
 


				     prev_marker_position = [trans[0], trans[1], trans[2]] 
				     if(goal_set):
                                             break
			   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                      print("exception")
				      rospy.sleep(0.1)

				  

                       if(goal_set):



		          object_grasped = 0
                          grasped = False



		          jt = JointTrajectory() 
			  jt.joint_names = ['head_1_joint', 'head_2_joint']
			  jtp = JointTrajectoryPoint()
			  jtp.positions = [0, -0.26] 
			  jtp.time_from_start = rospy.Duration(0.6) 
			  jt.points.append(jtp)
			  self.head_cmd.publish(jt)
                          print("head ready")
                          rospy.sleep(0.6)



			  try:
				  (trans,rot) = self.listener.lookupTransform('/base_footprint', '/human_goal', rospy.Time(0))
				  object_pose = geometry_msgs.msg.PoseStamped()
				  object_pose.header.frame_id = "aruco_marker_frame"


				  object_pose.pose.position.x= -0.1
				  object_pose.pose.orientation.w=1.0
				  object_name = "object"
				  self.scene.remove_attached_object(self.eef_link, name="object")
				  self.scene.remove_world_object("object")

				  self.scene.add_box(object_name, object_pose, size=(0.25, 0.08, 0.04))
			  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			  		print("exception")

                          self.write("Marker position " + str(trans) + " " + "Marker orientation " + str(rot))
                          self.object_pose(trans)
			  img_proc = PinholeCameraModel()
			  img_proc.fromCameraInfo(self.cam_info)
			  coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
                          if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
			    	          	              self.look_at(coordinate[0], coordinate[1], block=False)



                          grasped_object = 0 

                        #Test the 3 grasp points. If none of them ensure the robot grasps the object ask human to push the object
                          while (grasped_object == 0 and self.cout_grasp_human < 3):
		                send_data = Path()

				waypoints = []
				wpose = self.group.get_current_pose().pose
				wpose.position.x = trans[0] 
				wpose.position.y = trans[1] + 0.015
				wpose.position.z = trans[2]
				wpose.orientation.x = rot[0]
				wpose.orientation.y = rot[1]
				wpose.orientation.z = rot[2]
				wpose.orientation.w = rot[3]
		                
		                new_pose = PoseStamped()
		                new_pose.pose = wpose
		                send_data.poses.append(copy.deepcopy(new_pose))
				wpose.position.x +=  0.13
				waypoints.append(copy.deepcopy(wpose))
		                new_pose = PoseStamped()
		                new_pose.pose = wpose
		                send_data.poses.append(copy.deepcopy(new_pose))
		                self.message="moving"

				self.pub.publish(self.message)
		                self.write("Starts moving")
		                init_movement = datetime.now()
		                while (self.plan_feedback == "Done"):
		                      rospy.sleep(0.05)
		                msg = String()
		                msg.data = "request sent"
		                self.plan_done.publish(msg)
		           
		                self.send_waypoints.publish(send_data)
		                print(self.plan_feedback)

		                while (self.plan_feedback!= "Done"):


				       x, y = self.marker_xtion[0], self.marker_xtion[1]  #track object as robot picks it
				       if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
								    self.look_at_(x,y, block=False)
						                    rospy.sleep(0.01)
	  
		                      rospy.sleep(0.05)



		                print("plan computed")
		                msg = String()
		                msg.data = ""
		                self.plan_done.publish(msg)
		               

		                print(self.moveit_status)
		                response = self.check_alignment_y()
		                response_z = self.check_alignment_z() 
		                rospy.sleep(0.3)
                                #Test the second grasp point if first does not work
		                if(self.moveit_status == 4 or response == False or response_z == False ):

                                                self.cout_grasp_human += 1
                                               
                                                trans[0] = self.grasp_robot[self.cout_grasp_human]
                                else:

		                     grasped_object = 1

                         if (self.cout_grasp_human == 3 and grasped_object== 0): #If after 3 tries it does not manage to grasp, ask the human to push the object onto the gripper
                                    self.message ="Unsuccesfull"
                                    goal = PlayMotionGoal()
				    goal.motion_name = 'temporal_handover02' #Go to the offer pose
				    goal.skip_planning = False

                                    self.say_something("Please give me the object") 
                                    f = self.last_msg.wrench.force.z #initial force
                                    initial_force = f
                                    counter = 0
			            while ((initial_force - f) <= 1.2):  
						  rospy.sleep(0.5)
						  f = self.last_msg.wrench.force.z
						  self.head_cmd.publish(jt)
						  counter += 1 
						  if(counter > 10): 
						        self.say_something("Please give me the object") 
						        counter = 0 
					       
					      
				    if ((initial_force - f) >= 1.2): #Object has been pushed onto the gripper
                                          touch_links=['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_grasping_frame']
				          self.scene.attach_box(self.eef_link, "object", touch_links=touch_links)

                          else:

		                             self.scene.attach_box(self.eef_link, "object", touch_links=touch_links)






                          #Before closing the gripper look back at object, and switch to human face
		          x, y = self.marker_xtion[0], self.marker_xtion[1]
			  if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
							    self.look_at(x,y, block=False)
				                            rospy.sleep(0.01)

				                         

			  if(self.person):
						img_proc = PinholeCameraModel()
						img_proc.fromCameraInfo(self.cam_info)
						coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
						if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01):
						        self.look_at(coordinate[0],coordinate[1], block=False)
						        
                          #Ensure object is still before completing the grasp
                          while not object_stability:
                              object_stability = self.object_stable() 
                 
                          
			  jt = JointTrajectory()
			  jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']
			  jtp = JointTrajectoryPoint()
				   
			  jtp.positions = [0.025, 0.025] 
			  jtp.time_from_start = rospy.Duration(0.3) 
			  jt.points.append(jtp)
			  self.gripper_cmd.publish(jt)
                          rospy.sleep(0.3)



                          grasping_duration = datetime.now() - init_grasping 
                          self.write("Object transfer duration " + str(grasping_duration))
                          self.write("Close gripper, force " + str(self.last_msg.wrench.force))
                          print("ungrasped")
                          while (self.plan_feedback!= "Computing"):
                              rospy.sleep(0.05)
                          time_robot_grasp = datetime.now()

                          send_data = Path()
                          new_pose = PoseStamped()

			  waypoints = []
			  wpose = self.group.get_current_pose().pose
                          wpose.position.x -=0.01 #pull slightly 
                          waypoints.append(copy.deepcopy(wpose))

			  wpose.position.x = self.initial_pose[0] 
		          wpose.position.y = self.initial_pose[1]

			  wpose.orientation.x = self.initial_or[0]   
			  wpose.orientation.y = self.initial_or[1]
		          wpose.orientation.z = self.initial_or[2]
			  wpose.orientation.w = self.initial_or[3]
			  waypoints.append(copy.deepcopy(wpose))
                          new_pose.pose = wpose
                          send_data.poses.append(copy.deepcopy(new_pose))
			  wpose.position.z = self.initial_pose[2]
			  waypoints.append(copy.deepcopy(wpose))
                          new_pose.pose = wpose
                          send_data.poses.append(copy.deepcopy(new_pose))
                          msg = String()
                          msg.data = "request sent"
                          self.plan_done.publish(msg)
                          self.write("Start object placing")
                   
                          self.send_waypoints.publish(send_data)

                          while (self.plan_feedback!= "Done"): #Track object as it is placed
				       x, y = self.marker_xtion[0], self.marker_xtion[1]
				       if(x < -0.01 or x > 0.01 or y< -0.01 or y > 0.01):
								    self.look_at_object(x,y, block=False)

                              rospy.sleep(0.05)

                          print("plan computed")
                          msg = String()
                          msg.data = ""
                          self.plan_done.publish(msg)

                          place_time = datetime.now()
                          time_place_object = place_time - time_robot_grasp
                          self.write("Duration to place object :" + str(time_place_object))

                          jt = JointTrajectory()
		          jt.joint_names = ['head_1_joint', 'head_2_joint'] 
		          jtp = JointTrajectoryPoint()
		          jtp.positions = [0, -0.56] 
		          jtp.time_from_start = rospy.Duration(2.0) 
		          jt.points.append(jtp)
		          self.head_cmd.publish(jt)


		          print("leave object")

			  jt = JointTrajectory()
			  jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']
			  jtp = JointTrajectoryPoint()
				   
			  jtp.positions = [0.044, 0.044] #to close gripper
			  jtp.time_from_start = rospy.Duration(0.7) 
			  jt.points.append(jtp)
			  self.gripper_cmd.publish(jt)

		          rospy.sleep(0.7)
                          self.write("Gripper open")
                          print("ungrasped")
                          self.scene.remove_attached_object(self.eef_link, name="object")
                          while (self.plan_feedback!= "Computing"):
                              rospy.sleep(0.05)

                          send_data = Path()
                          new_pose = PoseStamped()

		          waypoints = []
		          wpose = self.group.get_current_pose().pose
		          wpose.position.x -= 0.16

			  waypoints.append(copy.deepcopy(wpose))
                          new_pose.pose = wpose
                          send_data.poses.append(copy.deepcopy(new_pose))

                          msg.data = "request sent"
                          self.plan_done.publish(msg)
                          self.write("Start retreat")
                          self.send_waypoints.publish(send_data)
                          print(self.plan_feedback)
                          while (self.plan_feedback!= "Done"): 
                                        rospy.sleep(0.05)

                          print("plan computed")
                          msg = String()
                          msg.data = ""
                          self.plan_done.publish(msg)
                          self.begin_position('grasp17') #go to initial position
           


                          robot_duration = datetime.now() - init_movement
                          self.write("Stop moving. Duration for robot movement " + str(robot_duration))

                          self.write("Duration from place to stop movement :" + str(datetime.now() -time_place_object ))



	       	          img_proc = PinholeCameraModel()
		          img_proc.fromCameraInfo(self.cam_info)
	            	  coordinate = img_proc.projectPixelTo3dRay((self.pos[0],self.pos[1]))
                          if(coordinate[0] < -0.01 or coordinate[0] > 0.01 or coordinate[1] < -0.01 or coordinate[1] > 0.01): 
			    	self.look_at(coordinate[0], coordinate[1], block=False)

                          self.scene.remove_world_object("object")

		          if (self.message == " "): 
                                self.message ="Succesfull"
                        
		          self.pub.publish(self.message)
		          print self.message
                          self.write(self.message)
		       
		          self.action = 'None'
                          self.message =" "
                          goal_set = False
                          count_human = 0
           
			  rospy.sleep(0.5)



                       

        




if __name__ == '__main__':
    rospy.init_node('robot_and_human_handover')

    ftp = Handover()
    rospy.spin()
                       
