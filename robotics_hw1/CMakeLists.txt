cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

find_package(
  catkin REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
    std_msgs
)

add_message_files(
  FILES
    MotorSpeed.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
)
