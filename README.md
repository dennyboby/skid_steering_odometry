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

## 2. Working

## 3. Setup

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

