
#include <skid_steering/skid_steering.h>

namespace skid_steering {

SkidSteering::SkidSteering()
        : base_frame_id_("base_link")
        , wheel_separation_(0)
        , wheel_radius_(0)
        , chi_(0)
        , integration_method_(0)
        , gear_ratio_(0)
        , odom_frame_id_("odom")
        , enable_odom_tf_(true)
        , set_x_(0)
        , set_y_(0)
        , set_theta_(0)
        , fl_wheel_speed_(0)
        , fr_wheel_speed_(0)
        , rl_wheel_speed_(0)
        , rr_wheel_speed_(0)
        , wheel_subscriber_timeout_(0)
        , reset_(false)
        , set_(false)  
        , sync_(SyncPolicy(10), sub_fl_, sub_fr_, sub_rl_, sub_rr_) {}

bool SkidSteering::init(ros::NodeHandle& nh) {

    nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);

    int velocity_rolling_window_size = 10;
    ROS_INFO("Velocity rolling window size of %d.", velocity_rolling_window_size);

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO("Base frame_id set to %s", base_frame_id_.c_str());

    nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO("Odometry frame_id set to %s", odom_frame_id_.c_str());

    nh.param("wheel_subscriber_timeout", wheel_subscriber_timeout_, wheel_subscriber_timeout_);
    ROS_INFO("Wheel subscriber timeout set to %f", wheel_subscriber_timeout_);

    // Robot parameters
    nh.param("wheel_separation", wheel_separation_, wheel_separation_);
    nh.param("wheel_radius", wheel_radius_, wheel_radius_);
    nh.param("chi", chi_, chi_);
    nh.param("integration_method", integration_method_, integration_method_);
    nh.param("gear_ratio", gear_ratio_, gear_ratio_);
    ROS_INFO("Robot parameters: \nwheel separation: %f\nwheel radius: %f\nchi: %f\nintegration method: %d\ngear ratio: %d", wheel_separation_, wheel_radius_, chi_, integration_method_, 
                gear_ratio_);

    // Set Robot parameters for odometery computation
    odometry_.setParams(wheel_separation_, wheel_radius_, chi_, integration_method_);

    setOdomPubFields(nh);

    // Initialize subscriber
    sub_fl_.subscribe(nh, "/motor_speed_fl", 1);
    sub_fr_.subscribe(nh, "/motor_speed_fr", 1);
    sub_rl_.subscribe(nh, "/motor_speed_rl", 1);
    sub_rr_.subscribe(nh, "/motor_speed_rr", 1);
    sync_.registerCallback(boost::bind(&SkidSteering::wheelCallback, this, _1, _2, _3, _4));

    // Initialize dynamic parameters
    DynamicParams dynamic_params;
    dynamic_params.integration_method  = integration_method_;
    dynamic_params.enable_odom_tf = enable_odom_tf_;

    dynamic_params_.writeFromNonRT(dynamic_params);

    // Initialize dynamic_reconfigure server
    SkidSteeringConfig config;
    config.integration_method  = integration_method_;
    config.enable_odom_tf = enable_odom_tf_;

    dyn_reconf_server_.setCallback(boost::bind(&SkidSteering::reconfCallback, this, _1, _2));

    // Initialize service
    reset_odom_service_ = nh.advertiseService("reset_odom", &SkidSteering::resetOdom, this);
    set_odom_service_ = nh.advertiseService("set_odom_pose", &SkidSteering::setOdomPose, this);

    odometry_.init(ros::Time::now());
    return true;
}

void SkidSteering::update(const ros::Time& time) {

    updateDynamicParams();
    odometry_.setParams(wheel_separation_, wheel_radius_, chi_, integration_method_);

    if (reset_)
    {
        odometry_.resetOdom();
        reset_ = false;
    }

    if (set_)
    {
        odometry_.setOdom(set_x_, set_y_, set_theta_);
        set_ = false;
    }

    if (wheel_subscriber_timeout_ > (time - current_callback_time_).toSec())
    {
        /// COMPUTE AND PUBLISH ODOMETRY
        odometry_.update(-rpmToMPerS(fl_wheel_speed_), rpmToMPerS(fr_wheel_speed_),
                -rpmToMPerS(rl_wheel_speed_), rpmToMPerS(rr_wheel_speed_), time);
    }
    else
    {
        ROS_INFO("Wheel callback timeout occured.");
    }

    /// Publish odometry message
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if (odom_pub_->trylock()) {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
    }

    // Populate odom message and publish on custom_odom topic
    if (custom_odom_pub_->trylock()) {
        custom_odom_pub_->msg_.odom.header.stamp = time;
        custom_odom_pub_->msg_.odom.pose.pose.position.x = odometry_.getX();
        custom_odom_pub_->msg_.odom.pose.pose.position.y = odometry_.getY();
        custom_odom_pub_->msg_.odom.pose.pose.orientation = orientation;
        custom_odom_pub_->msg_.odom.twist.twist.linear.x = odometry_.getLinear();
        custom_odom_pub_->msg_.odom.twist.twist.angular.z = odometry_.getAngular();
        custom_odom_pub_->msg_.integration_method = (integration_method_ == 0) ? "euler" : "rk";
        custom_odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock()) {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
    }
}

void SkidSteering::reconfCallback(skid_steering::SkidSteeringConfig& config, uint32_t level) {
    DynamicParams dynamic_params;
    dynamic_params.integration_method  = config.integration_method;
    dynamic_params.enable_odom_tf = config.enable_odom_tf;

    dynamic_params_.writeFromNonRT(dynamic_params);

    ROS_INFO("Dynamic Reconfigure integration_method: %d enable_odom_tf: %d", dynamic_params.integration_method, dynamic_params.enable_odom_tf);
}

void SkidSteering::updateDynamicParams() {
    // Retreive dynamic params:
    const DynamicParams dynamic_params = *(dynamic_params_.readFromRT());

    integration_method_  = dynamic_params.integration_method;
    enable_odom_tf_ = dynamic_params.enable_odom_tf;
}

void SkidSteering::setOdomPubFields(ros::NodeHandle& nh) {
    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 100));
    custom_odom_pub_.reset(new realtime_tools::RealtimePublisher<skid_steering::CustomOdom>(nh, "custom_odom", 100));
    custom_odom_pub_->msg_.odom.header.frame_id, odom_pub_->msg_.header.frame_id = odom_frame_id_;
    custom_odom_pub_->msg_.odom.child_frame_id, odom_pub_->msg_.child_frame_id = base_frame_id_;
    custom_odom_pub_->msg_.odom.pose.pose.position.z, odom_pub_->msg_.pose.pose.position.z = 0;
    custom_odom_pub_->msg_.odom.twist.twist.linear.y, odom_pub_->msg_.twist.twist.linear.y = 0;
    custom_odom_pub_->msg_.odom.twist.twist.linear.z, odom_pub_->msg_.twist.twist.linear.z = 0;
    custom_odom_pub_->msg_.odom.twist.twist.angular.x, odom_pub_->msg_.twist.twist.angular.x = 0;
    custom_odom_pub_->msg_.odom.twist.twist.angular.y, odom_pub_->msg_.twist.twist.angular.y = 0;

    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

float SkidSteering::rpmToMPerS(const float rpm){
    return (rpm * wheel_radius_ * 0.104719755) / gear_ratio_; // 0.104719755 = (2 * PI) / 60
}

void SkidSteering::wheelCallback(const robotics_hw1::MotorSpeedConstPtr& fl_msg, 
                                    const robotics_hw1::MotorSpeedConstPtr& fr_msg,
                                    const robotics_hw1::MotorSpeedConstPtr& rl_msg, 
                                    const robotics_hw1::MotorSpeedConstPtr& rr_msg){
    current_callback_time_ = ros::Time::now();
    fl_wheel_speed_ = fl_msg->rpm;
    fr_wheel_speed_ = fr_msg->rpm;
    rl_wheel_speed_ = rl_msg->rpm;
    rr_wheel_speed_ = rr_msg->rpm;
}

bool SkidSteering::resetOdom(skid_steering::ResetOdomPose::Request  &req, 
                     skid_steering::ResetOdomPose::Response &res){
    if (req.reset_odom){
        set_x_ = 0;
        set_y_ = 0;
        set_theta_ = 0;
        res.result = true;
        reset_ = true;
    }
    return true;
}

bool SkidSteering::setOdomPose(skid_steering::SetOdomPose::Request  &req, 
                     skid_steering::SetOdomPose::Response &res){
    set_x_ = req.x;
    set_y_ = req.y;
    set_theta_ = req.theta;
    res.result = true;
    set_ = true;
    return true;                               
}

}  // namespace skid_steering

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skid_steering_node");
    ros::NodeHandle nh;
    ros::Rate r(50);
    skid_steering::SkidSteering skid_steering;
    skid_steering.init(nh);

    while(nh.ok())
    {
        ros::spinOnce(); 
        skid_steering.update(ros::Time::now());
        r.sleep();
    }
    return 0;
}

