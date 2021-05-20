#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <skid_steering/SkidSteeringConfig.h>

void callback(skid_steering::SkidSteeringConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.integration_method);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dyanamic_test");

  dynamic_reconfigure::Server<skid_steering::SkidSteeringConfig> server;
  dynamic_reconfigure::Server<skid_steering::SkidSteeringConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Dyanamic_test node");
  ros::spin();
  return 0;
}