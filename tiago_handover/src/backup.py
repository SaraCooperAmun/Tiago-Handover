
#The script includes backup commands to be used when Tiago fails to grasp, or there is need to place Tiago's gripper back in its original position and pause the Matlab script. Input the following in order to to reinitialize Tiago,
#2) Stop Matlab script
#1) Return Tiago's gripper to initial position
#3) Restart Matlab script

#Other commands include
#4) Open gripper. Used for instance when the person starts to pull too soon
#5) Open gripper and retreat
#7) Indicate to Matlab action was unsuccesfull

#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient
from std_msgs.msg import String

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from nav_msgs.msg import Path 
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
import datetime
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import yaml
import io
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray


rospy.init_node('testing_experiment')
def moveit_state(msg):
	    
	moveit_status = msg.status.status  


                  

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
interpreter = moveit_commander.MoveGroupCommandInterpreter()
interpreter.execute("use arm")
group = moveit_commander.MoveGroupCommander("arm")
eef_link=group.get_end_effector_link()

stop_pub = rospy.Publisher('/stop', String, queue_size=1)
play_m_as = SimpleActionClient('/play_motion', PlayMotionAction) 
if not play_m_as.wait_for_server(rospy.Duration(20)):
	rospy.logerr("Could not connect to /play_motion AS")
	exit()
rospy.loginfo("Connected!")


pub = rospy.Publisher('trigger', String, queue_size=1)
moveit_status = 0
error = rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, moveit_state)
client = SimpleActionClient('/safe_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

while not rospy.is_shutdown():
    confirmcheck = raw_input('type go to continue: ')
    if confirmcheck == '1':

                       	scene.remove_world_object("table")
			scene.remove_world_object("right_restrict") 
			scene.remove_world_object("left_restrict") 
			scene.remove_world_object("top_restrict") 
			scene.remove_attached_object(eef_link, name="pringles")
			scene.remove_world_object("pringles")

                        goal = FollowJointTrajectoryGoal()

			goal.trajectory.joint_names= ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
                        jtp = JointTrajectoryPoint()
			data = rospy.get_param("mymotions")

			points= data.get('play_motion').get('motions').get('grasp17').get('points')[0]
         
                        jtp.positions = points.get('positions')

                       

			jtp.velocities = [0.12,0.12,0.12,0.12,0.12,0.12,0.12]
			jtp.time_from_start = rospy.Duration(1.5) #I can set the duration here
			goal.trajectory.points.append(jtp)
             

			client.send_goal(goal)

			client.wait_for_result()
                        pub.publish('not moving')
    elif confirmcheck == '2':
         message = 'stop'
         stop_pub.publish(message)

    elif confirmcheck == '3':
         message = 'go'
         stop_pub.publish(message)


    elif confirmcheck == '4': 

		goal = PlayMotionGoal()
		goal.skip_planning = False
		goal.motion_name = 'open_gripper'
		play_m_as.send_goal_and_wait(goal)

    elif confirmcheck == '5':



                scene.remove_world_object("table")
	        scene.remove_world_object("right_restrict") 
	        scene.remove_world_object("left_restrict") 
	        scene.remove_world_object("top_restrict") 
	        scene.remove_attached_object(eef_link, name="pringles")
	        scene.remove_world_object("pringles")
		goal = PlayMotionGoal()
		goal.skip_planning = False
		goal.motion_name = 'open_gripper'
		play_m_as.send_goal_and_wait(goal)
                rospy.sleep(0.5)
                goal = FollowJointTrajectoryGoal()

		goal.trajectory.joint_names= ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
                jtp = JointTrajectoryPoint()
		data = rospy.get_param("mymotions")

		points= data.get('play_motion').get('motions').get('grasp17').get('points')[0]
         
                jtp.positions = points.get('positions')

                       
			   # jtp.velocities = [0.5]
		jtp.velocities = [0.12,0.12,0.12,0.12,0.12,0.12,0.12]
		jtp.time_from_start = rospy.Duration(1.5) #I can set the duration here
		goal.trajectory.points.append(jtp)
             

		client.send_goal(goal)
                pub.publish('not moving')

    elif confirmcheck == '7':
              pub.publish('Unsuccesfull')

             

 
					   

    

