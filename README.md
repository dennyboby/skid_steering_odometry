# Skid Steering Odometry
The C++ ROS code is used to compute the odometry of skid steering mobile robot.

## Contents
- Skid Steering Odometry
  - [1. Introduction](#1-introduction)
  - [2. Working](#2-working)
  - [3. Setup](#3-setup)
  - [4. Run](#4-run)
    - [i. Setup parameters](#i-setup-parameters)
    - [ii. Visualize Rosbag](#ii-visualize-rosbag)
    - [iii. Run Skid Steering](#iii-run-skid-steering)

## 1. Introduction
This repo contains the C++ code to compute the odometry of a skid steering robot and visualize it on RViz. The code is impplemented on ROS requires ROS installation on computer to run it. The code is based on paper and ppt present in this [docs folder](https://github.com/dennyboby/skid_steering_odometry/tree/master/docs/document). 

## 2. Working
The skid_steering.cpp subcribes to four wheel velocity topic published by the rosbag present in [data folder](https://github.com/dennyboby/skid_steering_odometry/tree/master/docs/document). These velocity messages are filtered so that the velocities used are approximatelly syncronous in time. The [skid steering parameters](https://github.com/dennyboby/skid_steering_odometry/tree/master/skid_steering/config/skid_param.yaml) are loaded into parameter server so that skid_steering.cpp can use them to set odometery params and other params. The odometry.cpp uses either euler integration or runge-kuttan integration to compute the pose of the robot. This pose is published by the skid_steering.cpp via /odom topic and tf. A custom message containing odometry and integration method is also published on /custom_odom topic. Dynamic reconfigure can be used to change the integration method at runtime. Also two services(/reset_odom and /set_odom_pose) are available to reset the odometry to (0,0,0) and set the odometry to a specific pose. The odometry published by skid_steering.cpp is visualized on RViz and compared with manufacturers odometry(/scout_odom topic) published by the rosbag.

### RQT Graph

### TF Tree

## 3. Setup
The setup requires

## 4. Run

### i. Setup parameters

### ii. Visualize Rosbag

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering visualize_bag.launch

The above command runs bag1 as default. If you want to run a specific bag follow the command below.

    roslaunch skid_steering visualize_bag.launch bag_file_name:=<name of the bag>

Options avalilable for bag_file_name are bag2 and bag3.

### iii. Run Skid Steering

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering skid_steering.launch run_bag:=true

The above command runs bag1 as default. If you want to run a specific bag follow the command below.

    roslaunch skid_steering visualize_bag.launch bag_file_name:=<name of the bag> run_bag:=true

Options avalilable for bag_file_name are bag2 and bag3.

