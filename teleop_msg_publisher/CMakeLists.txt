cmake_minimum_required(VERSION 2.8.3)
project(teleop_msg_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  control_msgs
  kinova_driver
  kinova_msgs
)

catkin_python_setup()

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  control_msgs
)


catkin_package(
   CATKIN_DEPENDS kinova_driver kinova_msgs roscpp rospy message_runtime std_msgs geometry_msgs
  )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/teleop_msg_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/teleop_msg_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
