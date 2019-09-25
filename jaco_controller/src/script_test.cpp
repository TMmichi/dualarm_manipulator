#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/GetPlannerParams.h>
#include <moveit_msgs/SetPlannerParams.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <tf_conversions/tf_eigen.h>
#include <limits>

#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>
 
using namespace planning_scene_monitor;
using namespace robot_model_loader;

namespace{
    struct SharedStorage{
        SharedStorage(){}
        ~SharedStorage(){
            tf_buffer_.reset();
            state_monitors_.clear();
            models_.clear();
        }
        boost::mutex lock_;
        std::weak_ptr<tf2_ros::Buffer> tf_buffer_;
        std::map<std::string, robot_model::RobotModelWeakPtr> models_;
        std::map<std::string, CurrentStateMonitorWeakPtr> state_monitors_;
    };
 
    SharedStorage& getSharedStorage(){
        #if 0  // destruction of static storage interferes with static destruction in class_loader
        // More specifically, class_loader's static variables might be already destroyed
        // while being accessed again in the destructor of the class_loader-based kinematics plugin.
        static SharedStorage storage;
        return storage;
        #else  // thus avoid destruction at all (until class_loader is fixed)
        static SharedStorage* storage = new SharedStorage;
        return *storage;
        #endif
    }

    // Deleter that, additionally to T*, deletes another object too
    template <typename T, typename O>
    struct CoupledDeleter{
        const O* other_;
        CoupledDeleter(const O* other = nullptr) : other_(other){}
        void operator()(const T* p){
            delete other_;
            delete p;
        }
    };
}  // namespace


CurrentStateMonitorPtr getSharedStateMonitor(const robot_model::RobotModelConstPtr& robot_model, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer){
    return getSharedStateMonitor(robot_model, tf_buffer, ros::NodeHandle());
}

CurrentStateMonitorPtr getSharedStateMonitor(const robot_model::RobotModelConstPtr& robot_model,
                                            const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                            const ros::NodeHandle& nh){
    SharedStorage& s = getSharedStorage();
    boost::mutex::scoped_lock slock(s.lock_);
    auto it = s.state_monitors_.insert(std::make_pair(robot_model->getName(), CurrentStateMonitorWeakPtr())).first;
    CurrentStateMonitorPtr monitor = it->second.lock();
    if (!monitor){
        // if there was no valid entry, create one
        monitor.reset(new CurrentStateMonitor(robot_model, tf_buffer, nh));
        it->second = monitor;
    }
    return monitor;
}


ros::NodeHandle node_handle_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
robot_model::RobotModelConstPtr robot_model_;

//CurrentStateMontior with robot_model, tf_buffer, nh
planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_buffer_, node_handle_);


bool waitForCurrentState(double wait_time)
{
    double slept_time = 0.0;
    double sleep_step_s = std::min(0.05, wait_time / 10.0);
    ros::Duration sleep_step(sleep_step_s);
    while (!haveCompleteState() && slept_time < wait_time)
    {
        sleep_step.sleep();
        slept_time += sleep_step_s;
    }
    return haveCompleteState();
}

bool waitForCurrentState(const std::string& group, double wait_time)
{
    if (waitForCurrentState(wait_time))
        return true;
    bool ok = true;

    // check to see if we have a fully known state for the joints we want to record
    std::vector<std::string> missing_joints;
    if (!haveCompleteState(missing_joints))
    {
        const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
        if (jmg)
        {
        std::set<std::string> mj;
        mj.insert(missing_joints.begin(), missing_joints.end());
        const std::vector<std::string>& names = jmg->getJointModelNames();
        bool ok = true;
        for (std::size_t i = 0; ok && i < names.size(); ++i)
            if (mj.find(names[i]) != mj.end())
            ok = false;
        }
        else
        ok = false;
    }
    return ok;
}


void planning_scene_monitor::CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic){
    if (!state_monitor_started_ && robot_model_){
        joint_time_.clear();
        if (joint_states_topic.empty())
            ROS_ERROR("The joint states topic cannot be an empty string");
        else
            joint_state_subscriber_ = nh_.subscribe(joint_states_topic, 25, &CurrentStateMonitor::jointStateCallback, this);
        state_monitor_started_ = true;
        monitor_start_time_ = ros::Time::now();
        ROS_DEBUG("Listening to joint states on topic '%s'", nh_.resolveName(joint_states_topic).c_str());
    }
}


//Problem directs to here
bool getCurrentState_prob(robot_state::RobotStatePtr& current_state, double wait_seconds = 1.0){
    if (!current_state_monitor_){
        ROS_ERROR_NAMED("move_group_interface", "Unable to get current robot state");
        return false;
    } //Passed. no problem

    if (!current_state_monitor_->isActive())
        current_state_monitor_->startStateMonitor(); //잘 되었나??

    if (!current_state_monitor_->waitForCurrentState(ros::Time::now(), wait_seconds)){ //PROBLEM!!! -> current_state_monitor chk
        ROS_ERROR_NAMED("move_group_interface", "Failed to fetch current robot state");
        return false;
    }
    current_state = current_state_monitor_->getCurrentState();
    return true;
}



//PROBLEM!!!
robot_state::RobotStatePtr moveit::planning_interface::MoveGroupInterface::getCurrentState(double wait){
    robot_state::RobotStatePtr current_state;
    getCurrentState_prob(current_state, wait);
    return current_state; //<- need to receive
}

bool startStateMonitor(double wait){
    if (!current_state_monitor_){
        ROS_ERROR_NAMED("move_group_interface", "Unable to monitor current robot state");
        return false;
    }
    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
        current_state_monitor_->startStateMonitor();

    current_state_monitor_->waitForCompleteState(opt_.group_name_, wait);
    return true;
}

