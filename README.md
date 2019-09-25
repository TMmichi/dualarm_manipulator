# dualarm_manipulator

Manipulator control ROS package with moveit! for v-rep simulation / Local kinova jaco2 machine

Currently tested in Ubuntu 18.04 and ROS melodic

# 1. Installation

Since most of the ROS dependecies are included as a source in the repository, catkin_make will do the ROS package buliding process. Any errors occured during installation can be resolved through installing proper packages appeared in the error message. GOOD LUCK.

## 1-1.Preliminary

### V-rep
v-rep source can be downloaded from [here](http://www.coppeliarobotics.com/ubuntuVersions.html) and should be installed within the `/opt` folder. Installed directory can be changed, but should be matched with the vrep_path argument within the launch file: `vrep_jaco_bringup/launch/bringup.launch: vrep_path`
```
```

# 2. Usage

Manipulation of a real machine and one in the simulation are much alike from each other. 

## 2-1. Manipulator control within the v-rep simulation

### 2-1-1. Simulation environment bringup with ROS C++ Api

```
roslaunch vrep_jaco_bringup bringup.launch
```

`bringup.launch` file will launch a V-rep env with the scene file `jaco_table2.ttt`, and initialize the vrep_interface node (C++ api of V-rep).
Api script will initialize actionlib server side that can be connected with the moveit! planning instance.

Bringup launch file DOES NOT include manipulator URDF xacro.


#### Note:
- Prior to connect ROS-api of v-rep with the simulation environment itself, v-rep should be launched with the designated port number. Default port number has been set to `19997` in `action_client/src/VrepInterface.cpp` line number `38` with clientID_.

- If the api client node is running on a separate machine other than the machine with V-rep simulation, IP address should also be clarified, other than the default localhost (`127.0.0.1`)



### 2-1-2. Manipulator control node with moveit! in ROS node

```
roslaunch jaco_controller jaco_controller.launch
```

`jaco_controller.launch` includes most of the parameters required from the moveit! package with control parameters and launches the visualization node with RVIZ.

It also launches the ROS node with C++ script `jaco_controller.cpp` in folder `jaco_controller/src` which initializes the actionlib client side which communicates with the v-rep api server side.

By using Moveit! package, user does not have to consider action/state synchronization between the machine in the simulation and the controller node. Providing target gripper pose to the plan() method and calling move() within the move_group instance will publish rostopic type of `/name_of_the_remapped_machine/joint_trajectory/goal` from actionlib client side to the server side and wait till all of the goal joinst states in `joint_trajectory/goal` to be finished in simulation.



## 2-2. Manipulator control of a Real Machine

### 2-2-1. Jaco  bringup

```
roslaunch kinova_bringup kinova_robot.launch
```

### 2-2-2. Manipulator control node with moveit! in ROS node (real machine)

```
roslaunch jaco_controller_kinova jaco_controller_kinova.launch
```
As it is mentioned before, there is no significant differneces with the simulation ros control script.



### Reference

[moveit_source_code](https://github.com/ros-planning/moveit.git)

[kinova-ros_source_code](https://github.com/Kinovarobotics/kinova-ros.git)

[V-rep_api](https://github.com/JoshSong/jaco_ros_vrep.git)

[controller_script](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html)


