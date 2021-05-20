cmake_minimum_required(VERSION 3.0.2)
project(skid_steering)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  dynamic_reconfigure
  tf
  tf2
  tf2_ros
  geometry_msgs
  realtime_tools
  robotics_hw1
  message_generation
  message_filters
)

find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CustomOdom.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetOdomPose.srv
  SetOdomPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/SkidSteering.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamic_reconfigure
    tf
    tf2
    tf2_ros
    geometry_msgs
    realtime_tools
    nav_msgs
    robotics_hw1
    message_runtime
    message_filters
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(odometry
  src/odometry.cpp
)

add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(static_tf_publisher src/static_tf_publisher.cpp)
target_link_libraries(static_tf_publisher ${catkin_LIBRARIES})

add_executable(skid_steering src/skid_steering.cpp)
target_link_libraries(skid_steering ${catkin_LIBRARIES} odometry)
