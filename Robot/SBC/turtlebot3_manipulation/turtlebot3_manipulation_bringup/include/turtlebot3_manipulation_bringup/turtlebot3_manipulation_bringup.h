#ifndef TURTLEBOT3_MANIPULATION_BRINGUP_H
#define TURTLEBOT3_MANIPULATION_BRINGUP_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


class Turtlebot3ManipulationBringup
{
 public:
  Turtlebot3ManipulationBringup();
  ~Turtlebot3ManipulationBringup();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Publisher
  ros::Publisher joint_trajectory_point_pub_;
  ros::Publisher gripper_pub_;

  // ROS Subscriber
  ros::Subscriber display_planned_path_sub_;

  // ROS Server
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> arm_action_server_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> gripper_action_server_;

  // Callback Funcdtions
  void armActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &msg);
  void gripperActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &msg);
};

#endif //TURTLEBOT3_MANIPULATION_BRINGUP_H
