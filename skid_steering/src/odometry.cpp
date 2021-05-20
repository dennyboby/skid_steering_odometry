#include <skid_steering/odometry.h>

#include <boost/bind.hpp>

namespace skid_steering
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , wheel_radius_(0.0)
  , chi_(0.0)
  , integration_method_(0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateMethod, this, _1, _2, _3))
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double front_left_vel, double front_right_vel,
            double rear_left_vel, double rear_right_vel, const ros::Time &time)
  {
    /// Mean of same side wheels:
    const double left_wheel_vel  = (front_left_vel + rear_left_vel) * 0.5;
    const double right_wheel_vel = (front_right_vel + rear_right_vel) * 0.5;

    // const double lamda = ((-left_wheel_vel + right_wheel_vel) == 0)? 1 : 
    //                       (left_wheel_vel + right_wheel_vel)/(-left_wheel_vel + right_wheel_vel);

    // chi_ = computeICRCoefficient(lamda);
    
    /// Compute linear and angular velocity of robot:
    const double linear  = (right_wheel_vel + left_wheel_vel) * 0.5 ;
    const double angular = (right_wheel_vel - left_wheel_vel) / (wheel_separation_ * chi_);
    // ROS_INFO("Current chi value: %f", chi_);

    /// Time elapsed between previous and current iteration:
    const double dt = (time - timestamp_).toSec();

    /// Integrate odometry:
    integrate_fun_(linear, angular, dt);

    /// Set previous timestamp to current timestamp:
    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear);
    angular_acc_(angular);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }


  void Odometry::setParams(double wheel_separation, double wheel_radius, double chi,
                            int integration_method)
  {
    wheel_separation_   = wheel_separation;
    wheel_radius_  = wheel_radius;
    chi_ = chi;
    integration_method_ = integration_method;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }

  void Odometry::integrateRungeKutta(double linear, double angular, double dt)
  {
    const double direction = heading_ + (angular * 0.5 * dt);

    /// Runge-Kutta integration:
    x_       += linear * cos(direction) * dt;
    y_       += linear * sin(direction) * dt;
    heading_ += angular * dt;
  }

  void Odometry::integrateEuler(double linear, double angular, double dt)
  {
    /// Euler integration:
    x_       += linear * cos(heading_) * dt;
    y_       += linear * sin(heading_) * dt;
    heading_ += angular * dt;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateMethod(double linear, double angular, double dt)
  {
    if (integration_method_ == 0)
    {
      integrateEuler(linear, angular, dt);
    }
    else
    {
      integrateRungeKutta(linear, angular, dt);
    }
  }

  void Odometry::resetOdom()
  {
    x_ = 0;
    y_ = 0;
    heading_ = 0;
    ROS_INFO("Odometry reset to x: %f y: %f theta: %f", x_, y_, heading_);
  }

  void Odometry::setOdom(double x, double y, double theta)
  {
    x_ = x;
    y_ = y;
    heading_ = (theta * 3.14) / 180;
    ROS_INFO("Odometry set to x: %f y: %f theta: %f", x_, y_, heading_);
  }

  double Odometry::computeICRCoefficient(double lamda)
  {
    return 1 + (0.4728/(1 + (0.0538 * sqrt(abs(lamda)))));
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace skid_steering