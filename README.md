# Skid Steering Odometry
The C++ ROS code is used to compute the odometry of skid steering mobile robot.

### How to run dynamic reconfigure ###

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering dyanmic_test.launch

### How to run rosbag and visualize ###

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering visualize_bag.launch

The above command runs bag1 as default. If you want to run a specific bag follow the command below.

    roslaunch skid_steering visualize_bag.launch bag_file_name:=<name of the bag>

Options avalilable for bag_file_name are bag2 and bag3.

### How to run dynamic reconfigure ###

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering dyanmic_test.launch

### How to run skid steering ###

    cd <workspace>
    catkin build
    source devel/setup.bash
    roslaunch skid_steering skid_steering.launch run_bag:=true

The above command runs bag1 as default. If you want to run a specific bag follow the command below.

    roslaunch skid_steering visualize_bag.launch bag_file_name:=<name of the bag> run_bag:=true

Options avalilable for bag_file_name are bag2 and bag3.

