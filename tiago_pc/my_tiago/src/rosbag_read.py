#By: Sara Cooper
#Sample code to read rosbag data
#! /usr/bin/env python


import rosbag
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance, JointTrajectoryControllerState
from actionlib import SimpleActionClient, GoalStatus
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
if __name__ == '__main__':
   rospy.init_node('rosbag_processing')
   bag = rosbag.Bag('/home/sara/Desktop/2019001arm.bag')
   client = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
   client.wait_for_server()
   goal = FollowJointTrajectoryGoal()
   goal.trajectory.joint_names= ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
   for(topic, msg, t)in bag.read_messages():
					  jtp = JointTrajectoryPoint()

					  jtp=msg.actual

					  goal.trajectory.points.append(jtp)
   client.send_goal(goal)
   client.wait_for_result() 

