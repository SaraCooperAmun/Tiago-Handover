#!/usr/bin/env python

# By: Sara Cooper
#Human-to-robot handover, using play motion 




import rospy
from geometry_msgs.msg import WrenchStamped
from datetime import datetime
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

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

  
import time

marker_position = []
prev_marker_position = []
moved = False
time = 0.5
prev_speed = []
first_aruco = False
goal_set = False
height_from_table = 0
  

if __name__ == '__main__':

       rospy.init_node('listener', anonymous=True)
       listener = tf.TransformListener()
       marker = False
       moveit_commander.roscpp_initialize(sys.argv)

       scene = moveit_commander.PlanningSceneInterface()
       robot = moveit_commander.RobotCommander()
       interpreter = moveit_commander.MoveGroupCommandInterpreter()
       interpreter.execute("use arm")
       group = interpreter.get_active_group()
       eef_link=group.get_end_effector_link()
       scene.remove_attached_object(eef_link, name="pringles")
       scene.remove_world_object("pringles")
        		
       scene.remove_world_object("table")
       display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
       rate = rospy.Rate(1)
       play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
       if not play_m_as.wait_for_server(rospy.Duration(20)):
		rospy.logerr("Could not connect to /play_motion AS")
		exit()
       rospy.loginfo("Connected!")
       head_cmd = rospy.Publisher(
			'/head_controller/command', JointTrajectory, queue_size=1)
       jt = JointTrajectory() 
       jt.joint_names = ['head_1_joint', 'head_2_joint']  
       jtp = JointTrajectoryPoint()
       jtp.positions = [0, -0.56]
       jtp.time_from_start = rospy.Duration(2.0)
       jt.points.append(jtp)
       head_cmd.publish(jt)
       init = datetime.now()
       #Detect object and determine its speed
       while not goal_set:
           rospy.sleep(time)
           try:
                head_cmd.publish(jt)
           	(trans,rot) = listener.lookupTransform('/base_footprint', '/aruco_marker_frame', rospy.Time(0))
                if not (first_aruco):
                    table_pose = geometry_msgs.msg.PoseStamped()
                    table_pose.header.frame_id = "aruco_marker_frame"
	            table_pose.pose.position.x=-0.245                     table_pose.pose.position.y=-0.8
	            table_pose.pose.orientation.w=1.0
		    table_name = "table"
		    scene.add_box(table_name, table_pose, size=(0.1,2,2))
                    first_aruco = True
                    height_from_table = trans[0]
                marker_position = [trans[0], trans[1], trans[2]] 
                if(prev_marker_position):
       
                     velocity = [abs(marker_position[0] - prev_marker_position[0])/time, abs(marker_position[1] - marker_position[1])/time, abs(marker_position[2] - prev_marker_position[2])/time] 
                     if(velocity[2] > 0.01):
                               jt = JointTrajectory() 
			       jt.joint_names = ['head_1_joint', 'head_2_joint']
			       jtp = JointTrajectoryPoint()
			       jtp.positions = [0, -0.27]
			       jtp.time_from_start = rospy.Duration(2.0) 
			       jt.points.append(jtp)
			       head_cmd.publish(jt)
                     if(velocity[2] > 0.02):
		             moved = True


		     elif moved:
		              
		              object_pose = geometry_msgs.msg.PoseStamped()
			      object_pose.header.frame_id = "aruco_marker_frame"
			      object_pose.pose.position.x= -0.1
			      object_pose.pose.orientation.w=1.0
			      object_name = "object"
	
		              scene.add_box(object_name, object_pose, size=(0.25, 0.08, 0.04))
		              moved = False
                              goal_set = True

                prev_marker_position = [trans[0], trans[1], trans[2]]
                print moved
                
	   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		      continue
       #Once object speed is detected begin play motions to pregrasp, grasp, close, place, open gripper and retreat
       print("Goal set")
       print("Movement init time :" + str(datetime.now() - init))
       print("Start moving :" + str(datetime.now() - init))
       goal = PlayMotionGoal()
       goal.motion_name = 'pregrasp_4'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)
       print("Pregrasp position :" + str(datetime.now() - init))
       touch_links=['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_grasping_frame']
       scene.attach_box('arm_tool_link', "pringles", touch_links=touch_links)
       goal = PlayMotionGoal()
       goal.motion_name = 'grasp_object_4'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)
       print("Grasp time :" + str(datetime.now() - init))
       goal = PlayMotionGoal()
       goal.motion_name = 'close_gripper'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)
       
       goal = PlayMotionGoal()
       goal.motion_name = 'grasping_4_2'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)
       print("Leave object :" + str(datetime.now() - init))
       jt = JointTrajectory() 
       jt.joint_names = ['head_1_joint', 'head_2_joint']
       jtp = JointTrajectoryPoint()
       jtp.positions = [0, -0.34] 
       jtp.time_from_start = rospy.Duration(0.5) 
       jt.points.append(jtp)
       head_cmd.publish(jt)
       scene.remove_attached_object(eef_link, name="pringles")
       scene.remove_world_object("pringles")
       goal = PlayMotionGoal()
       goal.motion_name = 'open_gripper'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)

       print("Open gripper :" + str(datetime.now() - init))

       goal = PlayMotionGoal()
       goal.motion_name = 'grasping_4_3'
       goal.skip_planning = False
       play_m_as.send_goal_and_wait(goal)
       print("Retreat :" + str(datetime.now() - init))

  

