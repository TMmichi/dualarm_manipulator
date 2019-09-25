cmake_minimum_required(VERSION 2.8.3)
project(vrep_jaco_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vrep_jaco_bringup_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
