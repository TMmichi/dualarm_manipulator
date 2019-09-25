#include <ros/ros.h>
#include <random>
#include <chrono>
#include <algorithm>

// MoveIt
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
  return unif(rng)*2-1;
}

void setGripperState(moveit::planning_interface::MoveGroupInterface& gripper_group, double gripper_wideness){
  sensor_msgs::JointState gripper_states;
  gripper_states.header.frame_id = "";
  gripper_states.header.stamp = ros::Time::now();
  vector<double> gripper_value;
  ROS_INFO("Gripper value = %f",gripper_wideness);
  gripper_value.push_back(gripper_wideness/2*1.08);
  gripper_value.push_back(gripper_wideness/2*1.08);
  gripper_value.push_back(gripper_wideness/2*1.08);
  gripper_states.position = gripper_value;
  gripper_group.setJointValueTarget(gripper_states);
}


int main(int argc, char** argv)
{
  printf(MOVEIT_CONSOLE_COLOR_BLUE "JACO MAIN.\n" MOVEIT_CONSOLE_COLOR_RESET);
  ros::init(argc, argv, "jaco_kinova_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  double p_constant;
  int p_iter;
  string p_planning_group;
  string p_gripper_group;
  nh.param<double>("constant",p_constant,0.03);
  nh.param<int>("iter",p_iter,20);
  nh.param<string>("planning_group",p_planning_group,"arm");
  nh.param<string>("gripper_group",p_gripper_group,"gripper");

  ROS_INFO("GROUP: %s",p_planning_group.c_str());

  printf(MOVEIT_CONSOLE_COLOR_BLUE "Move_group setup within controller.\n" MOVEIT_CONSOLE_COLOR_RESET);
  static const string PLANNING_GROUP_ = p_planning_group;
  static const string GRIPPING_GROUP_ = p_gripper_group;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_);
  moveit::planning_interface::MoveGroupInterface gripper_group(GRIPPING_GROUP_);
  printf(MOVEIT_CONSOLE_COLOR_BLUE "Move_group setup finished.\n" MOVEIT_CONSOLE_COLOR_RESET);

  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
  const vector<string>& joint_names = joint_model_group->getVariableNames();
  vector<double> joint_values;
  joint_values = move_group.getCurrentJointValues();
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  const robot_state::JointModelGroup* joint_gripper_group =
    gripper_group.getCurrentState()->getJointModelGroup(GRIPPING_GROUP_);
  const vector<string>& gripper_names = joint_gripper_group->getVariableNames();
  vector<double> gripper_values;
  gripper_values = gripper_group.getCurrentJointValues();
  for (size_t i = 0; i < gripper_names.size(); ++i)
  {
    ROS_INFO("Gripper %s: %f", gripper_names[i].c_str(), gripper_values[i]);
  }
  
  geometry_msgs::Pose current_pose;
  current_pose = move_group.getCurrentPose().pose;
  double x,y,z;
  x = current_pose.position.x;
  y = current_pose.position.y;
  z = current_pose.position.z;
  ROS_INFO("pose_x: %f",x);
  ROS_INFO("pose_y: %f",y);
  ROS_INFO("pose_z: %f",z);

  tf2::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  ROS_INFO("orientation_r: %f",roll);
  ROS_INFO("orientation_p: %f",pitch);
  ROS_INFO("orientation_y: %f",yaw);

  geometry_msgs::Pose target_pose;

  for (int i=0;i<p_iter;i++){
    x += giverand() * p_constant;
    y += giverand() * p_constant;
    z += giverand() * p_constant;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    tf2::Quaternion orientation;
    roll += giverand() * p_constant;
    pitch += giverand() * p_constant;
    yaw += giverand() * p_constant;
    orientation.setRPY(roll,pitch,yaw);
    target_pose.orientation = tf2::toMsg(orientation);
    move_group.setPoseTarget(target_pose); //motion planning to a desired pose of the end-effector

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();

    double widness = giverand()+1;
    setGripperState(gripper_group,widness);
    gripper_group.move();
  }

  printf(MOVEIT_CONSOLE_COLOR_BLUE "Desired pose.\n" MOVEIT_CONSOLE_COLOR_RESET);
  ROS_INFO("pose_x: %f",x);
  ROS_INFO("pose_y: %f",y);
  ROS_INFO("pose_z: %f",z);
  ROS_INFO("pose_roll: %f",roll);
  ROS_INFO("pose_pitch: %f",pitch);
  ROS_INFO("pose_yaw: %f",yaw);

  printf(MOVEIT_CONSOLE_COLOR_BLUE "Current pose.\n" MOVEIT_CONSOLE_COLOR_RESET);
  current_pose = move_group.getCurrentPose().pose;
  tf2::Quaternion q_cur(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
  tf2::Matrix3x3 m_cur(q_cur);
  double roll_cur,pitch_cur,yaw_cur;
  m_cur.getRPY(roll_cur,pitch_cur,yaw_cur);
  ROS_INFO("orientation_r: %f",roll);
  ROS_INFO("orientation_p: %f",pitch);
  ROS_INFO("orientation_y: %f",yaw);
  ROS_INFO("pose_x: %f",current_pose.position.x);
  ROS_INFO("pose_y: %f",current_pose.position.y);
  ROS_INFO("pose_z: %f",current_pose.position.z);
  ROS_INFO("orientation_r: %f",roll_cur);
  ROS_INFO("orientation_p: %f",pitch_cur);
  ROS_INFO("orientation_y: %f",yaw_cur);
  //ros::waitForShutdown();
  return 0;
}