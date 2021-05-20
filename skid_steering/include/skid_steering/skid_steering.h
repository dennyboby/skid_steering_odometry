#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <robotics_hw1/MotorSpeed.h>
#include <skid_steering/odometry.h>
#include <skid_steering/CustomOdom.h>
#include <skid_steering/SkidSteeringConfig.h>
#include <skid_steering/ResetOdomPose.h>
#include <skid_steering/SetOdomPose.h>

namespace skid_steering {

class SkidSteering {
public:
    SkidSteering();

    /**
     * \brief Initialize
     */
    bool init(ros::NodeHandle& nh);

    /**
     * \brief Updates i.e. computes the odometry
     * \param time   Current time
     */
    void update(const ros::Time& time);

private:

    /// Wheel subscribers
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,
       robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> SyncPolicy;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fl_, sub_fr_, sub_rl_, sub_rr_;
    message_filters::Synchronizer<SyncPolicy> sync_;
    double wheel_subscriber_timeout_; // Timeout for wheel subscriber [sec]
    ros::Time current_callback_time_; // Last wheel callback time

    /// Motor values
    double fl_wheel_speed_, fr_wheel_speed_, rl_wheel_speed_, rr_wheel_speed_;

    /// Odometry related:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<skid_steering::CustomOdom>> custom_odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation
    double wheel_separation_;

    /// A skidding multiplication factor
    double chi_;

    /// Wheel radius (assuming it's the same for the all wheels):
    double wheel_radius_;

    /// The method used for integration
    int integration_method_;

    /// Gear ratio of wheels
    int gear_ratio_; // For example if gear ratio is 1:40 input 40

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// A struct to hold dynamic parameters set from dynamic_reconfigure server
    struct DynamicParams
    {
      int integration_method;
      bool enable_odom_tf;

      DynamicParams()
        : integration_method(0)
        , enable_odom_tf(true)
      {}
    };

    realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;

    /// Dynamic Reconfigure server
    typedef dynamic_reconfigure::Server<skid_steering::SkidSteeringConfig> ReconfigureServer;
    ReconfigureServer dyn_reconf_server_;

    /// Odom reset service
    ros::ServiceServer reset_odom_service_;
    ros::ServiceServer set_odom_service_;
    double set_x_, set_y_, set_theta_;
    bool reset_, set_;

private:
    /**
     * \brief Converts rpm to m/s
     * \param rpm Wheel speed in rpm
     */
    float rpmToMPerS(const float rpm);

    /**
     * \brief Wheel callback
     * \param fl_msg Front left wheel message
     * \param fr_msg Front right wheel message
     * \param rl_msg Rear left wheel message
     * \param rr_msg Rear right wheel message
     */
    void wheelCallback(const robotics_hw1::MotorSpeedConstPtr& fl_msg, 
                        const robotics_hw1::MotorSpeedConstPtr& fr_msg,
                        const robotics_hw1::MotorSpeedConstPtr& rl_msg, 
                        const robotics_hw1::MotorSpeedConstPtr& rr_msg);

    /**
     * \brief Sets the odometry publishing fields
     * \param nh Node handle
     */
    void setOdomPubFields(ros::NodeHandle& nh);

    /**
     * \brief Callback for dynamic_reconfigure server
     * \param config The config set from dynamic_reconfigure server
     * \param level not used at this time.
     * \see dyn_reconf_server_
     */
    void reconfCallback(skid_steering::SkidSteeringConfig& config, uint32_t level);

    /**
     * \brief Update the dynamic parameters in the RT loop
     */
    void updateDynamicParams();

    /**
     * \brief Callback for reset odom service
     * \param req The service request
     * \param res The service response
     * \return True if set
     */
    bool resetOdom(skid_steering::ResetOdomPose::Request  &req, 
                                skid_steering::ResetOdomPose::Response &res);

    /**
     * \brief Callback for set odom service which sets odom to x, y and theta
     * \param req The service request
     * \param res The service response
     * \return True if set
     */
    bool setOdomPose(skid_steering::SetOdomPose::Request  &req, 
                                skid_steering::SetOdomPose::Response &res);

};

}  // namespace skid_steering
