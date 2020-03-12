cmake_minimum_required(VERSION 2.8.3)
project(basicmodutil_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  rospy_message_converter
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES basicmodutil_pkg
  CATKIN_DEPENDS roscpp roslib rospy std_msgs rospy_message_converter
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bmod_node src/bm_node.cpp src/utilBM.cpp)
target_link_libraries(bmod_node ${catkin_LIBRARIES})

