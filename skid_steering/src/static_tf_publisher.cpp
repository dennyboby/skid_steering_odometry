#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "static_tf2_node");
  ros::NodeHandle nh;

  double initial_x, initial_y, initial_theta;
  nh.param("initial_x", initial_x, 0.0);
  nh.param("initial_y", initial_y, 0.0);
  nh.param("initial_theta", initial_theta, 0.0);
  ROS_INFO("Odom frame initial pose x: %f y: %f theta: %f", initial_x, initial_y, initial_theta);

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "odom";
  static_transformStamped.transform.translation.x = initial_x;
  static_transformStamped.transform.translation.y = initial_y;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, ((initial_theta * 3.14159) / 180));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);

  ros::spin();
  return 0;
};