#pragma once

#include <math.h>
#include "SimplePID.hpp"

namespace usyd_vrx {

class ThrustController
{
  public:
    /*!
    * Constructor.
    * @param thrust_config indicatines thrust configuration. "H" is differential, 
    *   "T" is lateral.
    * @param priority_yaw_range range of yaw values about the desired course
    *   yaw in which to prioritise angular velocity over linear velocity.
    * @param motor_cmd_limit maximum motor command value.
    * @param neg_scale_factor factor by which to scale negative thrust commands.
    * @param strafe_thrust value of thrust to use when strafing.
    * @param station_tolerance_ang angle error threshold under which station angle
    *   is considered aligned.
    */
    ThrustController(char thrust_config, float priority_yaw_range, float motor_cmd_limit, 
      float lateral_scale_x, float lateral_scale_y, float neg_scale_factor, 
      float strafe_thrust, float station_tolerance_ang);

    /*!
    * Destructor.
    */
    ~ThrustController();

    /*!
    * Configure linear thruster PID controller.
    * @param Kp proportional gain.
    * @param Ki integral gain.
    * @param Kd derivative gain.
    * @param max_integral maximum integral accumulation.
    * @param use_sim_time use simulator time (true) or real time (false)
    */
    void initLinearPID(float Kp, float Ki, float Kd, float max_integral, 
      bool use_sim_time);

    /*!
    * Configure angular thruster PID controller.
    * @param Kp proportional gain.
    * @param Ki integral gain.
    * @param Kd derivative gain.
    * @param max_integral maximum integral accumulation.
    * @param use_sim_time use simulator time (true) or real time (false)
    */
    void initAngularPID(float Kp, float Ki, float Kd, float max_integral, 
      bool use_sim_time);

    /*!
    * Resets PID variables.
    */
    void resetPIDs();

    /*!
    * Update controller with target velocity data.
    * @param linear_velocity desired linear x velocity.
    * @param angle desired yaw (rad).
    */
    void setTarget(double linear_velocity, double angle);

    /*!
    * Set ratio of thruster strength in order to strafe in direction given.
    * @param angle desired direction (rad) to strafe towards.
    */
    void setStrafeProportions(double angle);

    /*!
    * Update controller with current odometry data from vehicle.
    * @param linear_velocity current linear x velocity.
    * @param angle current yaw.
    */
    void setOdometry(double linear_velocity, double angle);

    /*!
    * Checks whether the desired station angle has been reached.
    * @return true if angle hit, false otherwise.
    */
    bool stationAngleHit();

    /*!
    * Use PIDs to set thruster values in traversal mode.
    * @param thrust_right float to store right thrust command in.
    * @param thrust_left float to store left thrust command in.
    * @param sim_time current ROS time.
    */
    void getControlSignalTraverse(
      float &thrust_right, float &thrust_left, double sim_time=0);

    /*!
    * Use angular PID to set thruster values in station keeping rotation.
    * @param thrust_right float to store right thrust command in.
    * @param thrust_left float to store left thrust command in.
    * @param thrust_lat float to store lateral thrust command in.
    * @param sim_time current ROS time.
    */
    void getControlSignalRotate(float &thrust_right, float &thrust_lat, 
      double sim_time=0);

      /*!
    * Set thruster values in station keeping strafing.
    * @param thrust_right float to store right thrust command in.
    * @param thrust_left float to store left thrust command in.
    * @param thrust_lat float to store lateral thrust command in.
    */
    void getStrafingThrust(float &thrust_right, float &thrust_left,
      float &thrust_lat);

  private:
    //! Letter indicating thrust configuration. "H" differential, "T" lateral.
    char thrust_config_;

    /*! 
    * Range of yaw values about the desired course yaw in which to 
    *   prioritise angular velocity over linear velocity.
    */
    float priority_yaw_range_;

    //! Maximum motor command value.
    float motor_cmd_limit_;

    //! Factor by which to scale lateral thrust commands for x strafing.
    float lateral_scale_x_;

    //! Factor by which to scale lateral thrust commands for y strafing.
    float lateral_scale_y_;

    //! Factor by which to scale negative thrust commands.
    float neg_scale_factor_;

    //! PID controller for linear velocity.
    usyd_vrx::SimplePID* linearPID_;

    //! PID controller for angular velocity.
    usyd_vrx::SimplePID* angularPID_;

    //! Current vessel yaw.
    float vessel_yaw_;

    //! Target station yaw.
    float station_yaw_;

    //! Angular error beneath which station is deemed rotationally aligned.
    float station_tolerance_ang_;

    //! Proportion of strafe speed to strafe at in relative x direction.
    float strafe_x_;

    //! Proportion of strafe speed to strafe at in relative y direction.
    float strafe_y_;

    //! Constant thrust value to use for strafing.
    float strafe_thrust_;

    /*!
    * Constrain thrust to motor command limit and negative scaling.
    * @param thrust value of thrust command.
    */
    double constrainThrust(double thrust);
};

}