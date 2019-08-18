#include <vector>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <nav_msgs/Path.h>
#include <string>
#include <std_msgs/String.h>
using namespace std;



std::vector<geometry_msgs::Pose> waypoints;

string plan_status;
void count_cb(const nav_msgs::Path& msg )
{
   //Reads waypoints send by main.py script, usually the pregrasp and grasp positions
        std::vector<geometry_msgs::Pose> curWaypoints;  

        for (int i = 0; i < msg.poses.size(); ++i)
         {
             curWaypoints.push_back(msg.poses[i].pose);
             ROS_INFO_STREAM(msg.poses[i].pose);

         }
        waypoints = curWaypoints;
         
        

}

 




void plan_cb(const std_msgs::String::ConstPtr& msg)
{
   
  plan_status =  msg->data;

 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_trajectory");
  ros::NodeHandle node;
 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber move_points= node.subscribe("/my_waypoints", 1,count_cb);
  ros::Publisher plan_ok = node.advertise<std_msgs::String>("/plan_done", 1); //Publish whether plan has been computed or not
  ros::Subscriber plan_now = node.subscribe("/plan_request", 1, plan_cb); //Read a new motion execution request from main.py
  moveit::planning_interface::MoveGroup group("arm");
  group.setPoseReferenceFrame("/base_footprint");
  group.setPlanningTime(5);

  // Create a trajectory (vector of Eigen poses)
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  Eigen::Affine3d pose (Eigen::Affine3d::Identity());

  // Copy the vector of Eigen poses into a vector of ROS poses
  ros::Rate loop_rate(1);

  while (ros::ok())  {
      
         ros::Duration(0.2).sleep();


     ROS_INFO_STREAM(plan_status);
     if(plan_status == "request sent") {
         std_msgs::String msg;

         msg.data = "Computing";
         plan_ok.publish(msg);
         ros::Duration(0.2).sleep();
         std::vector<geometry_msgs::Pose> way_points_msg;
         way_points_msg = waypoints;
  
         ROS_INFO_STREAM("list of waypoints" << waypoints.size());
         moveit_msgs::RobotTrajectory trajectory_msg;
         //Plan cartesian path trajectory with MoveIt, without time parameterization
 
         double fraction = group.computeCartesianPath(way_points_msg,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, true);

         ROS_INFO_STREAM("before processing" <<  trajectory_msg);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
         robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");

  // Second get a RobotTrajectory from trajectory
         rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
 
  // Thrid create a IterativeParabolicTimeParameterization object
         trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
         iptp.computeTimeStamps(rt);

  // Get RobotTrajectory_msg from RobotTrajectory, including the trajectory after post-processing
         rt.getRobotTrajectoryMsg(trajectory_msg);

         ROS_INFO_STREAM("after processing" << trajectory_msg);

  // Execute trajectory with MoveIt
         ros::ServiceClient executeKnownTrajectoryServiceClient = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");
         moveit_msgs::ExecuteKnownTrajectory srv;

         srv.request.wait_for_execution = true;
         srv.request.trajectory = trajectory_msg;
         executeKnownTrajectoryServiceClient.call(srv);


   //Notify the main.py node that the path has been executed
         msg.data = "Done";
         plan_ok.publish(msg);
         ros::Duration(2).sleep();
         loop_rate.sleep();
         plan_status = "done";

         msg.data = "Computing";
         plan_ok.publish(msg);



     }

}
        
         return 0;
}
