#include <ros/ros.h>
#include <random>
#include <chrono>
#include <algorithm>

// MoveIt
#include "action_client/VrepInterface.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/macros/console_colors.h>

using namespace std;

double giverand(){
  mt19937_64 rng;
  // initialize the random number generator with time-dependent seed
  uint64_t timeSeed = chrono::high_resolution_clock::now().time_since_epoch().count();
  seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
  rng.seed(ss);
  // initialize a uniform distribution between 0 and 1
  uniform_real_distribution<double> unif(0, 1);
  return unif(rng);
}


int main(int argc, char** argv)
{
  printf(MOVEIT_CONSOLE_COLOR_BLUE "JACO MAIN.\n" MOVEIT_CONSOLE_COLOR_RESET);
  ros::init(argc, argv, "jaco_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  double p_constant;
  int p_iter;
  nh.param("/jaco_ros_controller/constant",p_constant,0.03);
  nh.param("/jaco_ros_controller/iter",p_iter,20);

  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("jaco_driver/trajectory_controller",true);
  //ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  //ac.waitForServer();
  //ROS_INFO("Action server started");

  printf(MOVEIT_CONSOLE_COLOR_BLUE "Move_group setup within controller.\n" MOVEIT_CONSOLE_COLOR_RESET);
  static const string PLANNING_GROUP_ = "jaco_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_);
  printf(MOVEIT_CONSOLE_COLOR_BLUE "Move_group setup finished.\n" MOVEIT_CONSOLE_COLOR_RESET);

  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
  ROS_INFO("Joint states called");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  joint_values = move_group.getCurrentJointValues();
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  geometry_msgs::Pose current_pose;
  current_pose = move_group.getCurrentPose().pose;
  ROS_INFO("pose_x: %f",current_pose.position.x);
  ROS_INFO("pose_y: %f",current_pose.position.y);
  ROS_INFO("pose_z: %f",current_pose.position.z);
  ROS_INFO("orientation_w: %f",current_pose.orientation.w);
  ROS_INFO("orientation_x: %f",current_pose.orientation.x);
  ROS_INFO("orientation_y: %f",current_pose.orientation.y);
  ROS_INFO("orientation_z: %f",current_pose.orientation.z);

  for (int i=0;i<p_iter;i++){
    geometry_msgs::Pose target_pose;
    target_pose.position.x = current_pose.position.x + giverand() * p_constant;
    target_pose.position.y = current_pose.position.y + giverand() * p_constant;
    target_pose.position.z = current_pose.position.z + giverand() * p_constant;
    target_pose.orientation.w = current_pose.orientation.w;
    move_group.setPoseTarget(target_pose); //motion planning to a desired pose of the end-effector

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    /*
    joint_values = move_group.getCurrentJointValues();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = "root";
    ac.sendGoalAndWait(goal);*/

  }

  current_pose = move_group.getCurrentPose().pose;
  ROS_INFO("pose_x: %f",current_pose.position.x);
  ROS_INFO("pose_y: %f",current_pose.position.y);
  ROS_INFO("pose_z: %f",current_pose.position.z);
  ROS_INFO("orientation_w: %f",current_pose.orientation.w);
  ROS_INFO("orientation_x: %f",current_pose.orientation.x);
  ROS_INFO("orientation_y: %f",current_pose.orientation.y);
  ROS_INFO("orientation_z: %f",current_pose.orientation.z);
  //ros::waitForShutdown();
  return 0;
}