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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define JOINT_NAMES_ "jaco_joint_1", 'jaco_joint_2','jaco_joint_3','jaco_joint_4','jaco_joint_5','jaco_joint_6','jaco_joint_finger_1','jaco_joint_finger_2','jaco_joint_finger_3','jaco_joint_finger_tip_1','jaco_joint_finger_tip_2','jaco_joint_finger_tip_3'
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

void set_test_msg(control_msgs::FollowJointTrajectoryGoal& goal, double time_dur){
  ros::Duration start_duration(0.001);
  goal.trajectory.points.clear();
  for (int j=0; j<12; j++){
      goal.trajectory.joint_names.push_back("");
    }
    for (int k=0; k<10; k++){
      trajectory_msgs::JointTrajectoryPoint test_points;
      for (int l=0; l<12; l++){
        test_points.positions.push_back(giverand());
        test_points.velocities.push_back(0);
        test_points.accelerations.push_back(0);
      }
      start_duration += ros::Duration(time_dur);
      test_points.time_from_start = start_duration;    
      goal.trajectory.points.push_back(test_points);
    }
  goal.goal_time_tolerance.isZero();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("jaco/joint_trajectory_action",true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();
  ROS_INFO("Action server started");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("jaco_arm");
  const vector<string>& joint_names = joint_model_group->getVariableNames();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  vector<double> joint_values;

  for (int i=0;i<5;i++){
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = "root";
    set_test_msg(goal,10);
    ac.sendGoalAndWait(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      /*
      try{
        ROS_INFO("In joint value setting");
        joint_values.clear();
        for(double value : goal.trajectory.points[-1].positions){
          ROS_INFO("%f", value);
          joint_values.push_back(value);
        }
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      }catch(exception& e){
        ROS_INFO("%s",e.what());
      }*/
    }else
      ROS_INFO("Action did not finish before the time out.");
  }
  


  //kinematic_state->setToDefaultValues();
  //kinematic_state->setJointPositions();

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("jaco_link_2");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");


  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  return 0;
}