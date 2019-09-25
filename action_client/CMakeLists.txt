cmake_minimum_required(VERSION 2.8.3)
project(action_client)

# Set make arguments
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  control_msgs
  actionlib
  vrep_api
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    control_msgs
    actionlib
    vrep_api
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(vrep_interface src/VrepInterface.cpp)
target_link_libraries(vrep_interface ${catkin_LIBRARIES})