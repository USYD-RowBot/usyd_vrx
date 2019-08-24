#pragma once

#include <math.h>
#include "vrx_control_2/SimplePID.hpp"

namespace usyd_vrx {

class ThrustController
{
  public:
    /*!
    * Constructor.
    * @param priority_yaw_range range of yaw values about the desired course
    *   yaw in which to prioritise angular velocity over linear velocity.
    * @param motor_cmd_limit maximum motor command value.
    * @param neg_scale_factor factor by which to scale negative thrust commands.
    */
    ThrustController(float priority_yaw_range, float motor_cmd_limit,
      float neg_scale_factor);

    /*!
    * Configure linear thruster PID controller.
    */
    void initLinearPID(float Kp, float Ki, float Kd, float max_integral);

    /*!
    * Configure angular thruster PID controller.
    */
    void initAngularPID(float Kp, float Ki, float Kd, float max_integral);

    /*!
    * Update controller with target velocity data.
    */
    void setTarget(double linear_velocity, double angle);

    /*!
    * Update controller with current odometry data from vehicle.
    */
    void setOdometry(double linear_velocity, double angle);

    /*!
    * Calculate next PID outputs for thruster controls.
    * @param thrust_right float to store right thrust command in.
    * @param thrust_left float to store left thrust command in.
    * @param sim_time current ROS time.
    */
    void computeControlSignals(float &thrust_right, float &thrust_left,
      double sim_time=0);

  private:
    /*! 
    * Range of yaw values about the desired course yaw in which to 
    *   prioritise angular velocity over linear velocity.
    */
    float priority_yaw_range_;

    //! Maximum motor command value.
    float motor_cmd_limit_;

    //! Factor by which to scale negative thrust commands.
    float neg_scale_factor_;

    //! PID controller for linear velocity.
    usyd_vrx::SimplePID* linearPID_;

    //! PID controller for angular velocity.
    usyd_vrx::SimplePID* angularPID_;

    /*!
    * Constrain thrust to motor command limit and negative scaling.
    * @param thrust value of thrust command.
    */
    double constrainThrust(double thrust);
};

}