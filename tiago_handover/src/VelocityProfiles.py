#!/usr/bin/env python

#By: Sara Cooper
#Code used to test different kinds of grasping and motion planning for Tiago

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

class ForceTorquePlay(object):
    def __init__(self):
        self.patient_id = '2019001' 
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
        self.heightType = '03'

        self.marker_xtion = []

        self.force_torque_sub = rospy.Subscriber('/wrist_ft',
                                                 WrenchStamped,
                                                 self.force_torque_cb,
                                                 queue_size=1)

        self.pub = rospy.Publisher('/trigger', String, queue_size=1)
        self.plan_done= rospy.Subscriber('/done', String, self.plan_state)

        
	self.scene = moveit_commander.PlanningSceneInterface()
	self.robot = moveit_commander.RobotCommander()
	self.interpreter = moveit_commander.MoveGroupCommandInterpreter()
	self.interpreter.execute("use arm")
        self.group = moveit_commander.MoveGroupCommander("arm")
	self.eef_link=self.group.get_end_effector_link()
        self.scene.remove_attached_object(self.eef_link, name="pringles")
        self.scene.remove_world_object("pringles")
        		
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
	self.head_cmd = rospy.Publisher(
				'/head_controller/command', JointTrajectory, queue_size=1)
	self.arm_controller = rospy.Publisher(
				'/arm_controller/command', JointTrajectory, queue_size=1)
        self.message = " "

	self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)   
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
        self.my_rosbag = rosbag.Bag('/home/pal/sara_ws/patient_data/patientriggers'+self.patient_id+'block1.bag', 'w')
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
	self.object_grasped = 0
	self.near_object = 0
	self.move_start = 0
        self.initial_pose = []
        self.initial_or = []
	self.hand_up = False
	self.hand_forward = False
        self.environment_set = False
	self.grasped = False
        self.rest_pose = []
        self.first = True
        self.initial_torque = []
        self.torque_after_grasp = []
        self.object_weight_x = 0
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


    def aruco_xtion_tf(self, msg):
       self.marker_xtion = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
      

    def plan_state(self, msg):	  
	     self.plan_computed= msg.data


    def gripper_state(self, msg):
        self.gripper_joints = msg.actual.positions

    def arm_state(self,msg):
          self.arm_joints = msg


    def torso_state(self, msg):
           self.torso_joints = msg


    def moveit_state(self, msg):
	    
	     self.moveit_status = msg.status.status     
         

    def goto_cartesian(self, trans, rot):

			waypoints = []
			wpose = self.group.get_current_pose().pose
			wpose.position.x = trans[0] 
			wpose.position.y = trans[1]
			wpose.position.z = trans[2]
			wpose.orientation.x = rot[0]
			wpose.orientation.y = rot[1]
			wpose.orientation.z = rot[2]
			wpose.orientation.w = rot[3]
			waypoints.append(copy.deepcopy(wpose))

			(plan, fraction) = self.group.compute_cartesian_path(
								       waypoints,   # waypoints to follow
								       0.02,        # eef_step
								       0.0,True)    
                        self.group.execute(plan, wait=True)


    def run(self):

        marker = False          
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.trajectory_test()
            r.sleep()

    def trajectory_test(self):
        self.scene.remove_attached_object(self.eef_link, name="object")
        self.scene.remove_world_object("object")
        #Define how to perform trajectory planning for robot-to-human grasping. Cartesian = True indicates cartesian path planning,
        #smoothed = True includes post-processing to apply time parameterization with external node. 
        cartesian = True
        smoothed = True
        
        
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
                        if not cartesian:
                                 #Compute trajectory by specyfying end-effector goal directly
				pose_target = geometry_msgs.msg.Pose()
				pose_target.orientation.x = rot[0]
				pose_target.orientation.y = rot[1]
				pose_target.orientation.z = rot[2]
				pose_target.orientation.w = rot[3]
				pose_target.position.x = trans[0]
				pose_target.position.y = trans[1]
				pose_target.position.z = trans[2]
				self.group.set_pose_target(pose_target)
		                self.message="moving"
				self.pub.publish(self.message)
		                self.write("Starts moving")
		                init_movement = datetime.now()
                                plan_time_start = datetime.now()
		                self.group.set_planner_id("ESTkConfigDefault")
		                plan = self.group.plan()     
		                plan_time_start = datetime.now()
		                plan_execute = datetime.now() - plan_time_start
		                print("Time for plan", plan_execute)
		                self.group.go()
		                rospy.sleep(20)
		                self.interpreter.execute("go forward 0.09")
		                print("completed path time", datetime.now() - plan_time_start)
	

                        else:
                                plan_time_start = datetime.now()
                                if smoothed:             #Compute cartesian path planning with smoothing
                                    send_data = Path()

                                    waypoints = []
                                    wpose = self.group.get_current_pose().pose
                                    waypoints.append(copy.deepcopy(wpose))
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
                                    wpose.position.x +=  0.1
                                    waypoints.append(copy.deepcopy(wpose))
                                    new_pose = PoseStamped()
                                    new_pose.pose = wpose
                                    send_data.poses.append(copy.deepcopy(new_pose))
                                    self.message="moving"
                                    print self.message
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
                                          rospy.sleep(0.05)

                                #Compute cartesian path planning without smoothing
                                else:
                      
                                        (plan, fraction) = self.group.compute_cartesian_path(
                                                                                       waypoints,   # waypoints to follow
                                                                                       0.03,        # eef_step
                                                                                       0.0,True)         # jump_threshold

                                   
                                         self.group.execute(plan, wait=True)
                                   

                                print("completed path time", datetime.now() - plan_time_start)

     


                                            



			  rospy.sleep(0.5)



                       

        




if __name__ == '__main__':
    rospy.init_node('force_torque_play')

    ftp = ForceTorquePlay()
    rospy.spin()
                       
