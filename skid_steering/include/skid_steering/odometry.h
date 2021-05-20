#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace skid_steering
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels velocity
     * \param front_left_vel  Front left wheel velocity [m/s]
     * \param front_right_vel  Front left wheel velocity [m/s]
     * \param rear_left_vel  Front left wheel velocity [m/s]
     * \param rear_right_vel  Front left wheel velocity [m/s]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(double front_left_vel, double front_right_vel,
            double rear_left_vel, double rear_right_vel, const ros::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief Resets odom x, y, heading to zero
     */
    void resetOdom();

    /**
     * \brief Sets odom x, y, heading to given values
     * \param x  x value to be set [m]
     * \param y  y value to be set [m]
     * \param theta  orientation to be set [degree]
     */
    void setOdom(double x, double y, double theta);

    /**
     * \brief Sets the odometry parameters
     * \param wheel_separation   Separation between left and right wheels [m]
     * \param wheel_radius  Wheel radius [m]
     * \param chi A skidding multiplication factor
     * \param integration_method Method to use for integration (0 for Euler and 1 for RungeKutta)
     */
    void setParams(double wheel_separation, double wheel_radius, double chi, int integration_method);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using Runge-Kutta
     * \param linear  Linear  velocity   [m/s]
     * \param angular Angular velocity [rad/s]
     * \param dt The time difference between previous and current iteration
     */
    void integrateRungeKutta(double linear, double angular, double dt);

    /**
     * \brief Integrates the velocities (linear and angular) using Euler
     * \param linear  Linear  velocity   [m/s]
     * \param angular Angular velocity [rad/s]
     * \param dt The time difference between previous and current iteration
     */
    void integrateEuler(double linear, double angular, double dt);

    /**
     * \brief Integrates the velocities (linear and angular) using the specified method
     * \param linear  Linear  velocity   [m/s]
     * \param angular Angular velocity [rad/s]
     * \param dt The time difference between previous and current iteration
     */
    void integrateMethod(double linear, double angular, double dt);

    /**
     * \brief Computes the ICR coefficient
     * \param lamda Nondimensional path curvature variable
     * \return ICR coefficient
     */
    double computeICRCoefficient(double lamda);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

    /// Parameters:
    double wheel_separation_; //   [m]
    double wheel_radius_;     //   [m]
    double chi_;              //   [m]
    int integration_method_;  //   [0 or 1]

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
}