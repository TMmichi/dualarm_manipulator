# Dualarm_manipulator

Manipulator controlling ROS package with moveit! for V-rep simulation / Local kinova jaco2 machine

Currently tested in Ubuntu 18.04 and ROS melodic

## 1. Installation

Installation in Ubuntu 18.04 with ROS melodic is recommended (since other version of ROS or ubuntu distro were not tested)

### 1-1. Preliminary

#### V-rep
V-rep source can be downloaded from [here](http://www.coppeliarobotics.com/ubuntuVersions.html) and should be installed within the `/opt` folder. Installed location can be varied, but should be matched with the vrep_path argument within the launch file: `vrep_jaco_bringup/launch/bringup.launch: vrep_path`

### 1-2. Moveit installation

```
sudo apt-get install ros-<distro>-moveit-core
sudo apt-get install ros-<distro>-moveit-ros
sudo apt-get install ros-<distro>-rviz-visual-tools
sudo apt-get install ros-<distro>-moveit-visual-tools
sudo apt-get install ros-<distro>-ompl
sudo apt-get install ros-<distro>-trac-ik-kinematics-plugin
```
##### Note
- If your libqt5x11extras5 version is greater than 5.5.1-3build1, you should downgrade your libqt with command
```
sudo apt-get install libqt5x11extras5=5.5.1-3build1
```
  in order to install ros-<distro>-rviz-visual-tools
  
### 1-3. Build & Source repo

Build your repo with `catkin_make` command in the directory where your `src` folder is located.
```bash
cd catkin_ws/src
git clone https://github.com/TMmichi/dualarm_manipulator.git
cd ..
catkin_make
```

After building your repo, source `setup.bash` file in your project devel folder to the `.bashrc`.
```bash
echo "source ~YOUR_PROJECT_FOLDER/devl/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Usage

Manipulation of a real machine and one in the simulation are much alike from each other. 

### 2-1. Manipulator control within the V-rep simulation

#### 2-1-1. Simulation environment bringup with ROS C++ Api

```
roslaunch vrep_jaco_bringup bringup.launch
```

`bringup.launch` file will launch a V-rep env with the scene file `jaco_table2.ttt`, and initialize the vrep_interface node (C++ api of V-rep).
Api script will initialize actionlib server side that can be connected with the moveit! planning instance.

Bringup launch file DOES NOT include manipulator URDF xacro.


##### Note:
- Prior to connect ROS-api of V-rep with the simulation environment itself, V-rep should be launched with the designated port number. Default port number has been set to `19997` in `action_client/src/VrepInterface.cpp` line number `38` with clientID_.

- If the api client node is running on a separate machine other than the machine with V-rep simulation, IP address should also be clarified, other than the default localhost (`127.0.0.1`)



#### 2-1-2. Manipulator control node with moveit! in ROS node

```
roslaunch jaco_controller jaco_controller.launch
```

`jaco_controller.launch` includes most of the parameters required from the moveit! package with control parameters and launches the visualization node with RVIZ.

It also launches the ROS node with C++ script `jaco_controller.cpp` in folder `jaco_controller/src` which initializes the actionlib client side which communicates with the v-rep api server side.

By using Moveit! package, user does not have to consider action/state synchronization between the machine in the simulation and the controller node. Providing target gripper pose to the `plan()` method and calling `move()` within the `move_group` instance will publish rostopic with name and type in `/USER_DEFINED_NAME/joint_trajectory/goal` and `control_msgs/FollowJointTrajectoryActionGoal` respectively. Published topic from move_gropup instance in actionlib client side will then be transfered to the server side and wait till all of the goal joint states in `joint_trajectory/goal` to be finished in the simulation.



### 2-2. Manipulator control of a Real Machine

#### 2-2-1. Jaco  bringup

```
roslaunch kinova_bringup kinova_robot.launch
```

#### 2-2-2. Manipulator control node with moveit! in ROS node (real machine)

```
roslaunch jaco_controller_kinova jaco_controller_kinova.launch
```
As it is mentioned before, there is no significant differnece with the simulation ros control script.



#### Reference

[kinova-ros_source_code](https://github.com/Kinovarobotics/kinova-ros.git) from github@kinova-robotics

[V-rep_api](https://github.com/JoshSong/jaco_ros_vrep.git) from github@JoshSong

[controller_script](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html) from moveit_tutorial


