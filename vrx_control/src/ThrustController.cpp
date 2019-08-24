#include "vrx_control_2/ThrustController.hpp"

static int signf(float value)
{
  return (value > 0) - (value < 0);
}

namespace usyd_vrx {

ThrustController::ThrustController(float priority_yaw_range, 
  float motor_cmd_limit, float neg_scale_factor):

  priority_yaw_range_(fabs(priority_yaw_range)), 
  motor_cmd_limit_(fabs(motor_cmd_limit)),
  neg_scale_factor_(fabs(neg_scale_factor))
{}

void ThrustController::initLinearPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{  
  SimplePID::TIME_MODE time_mode;
  if (use_sim_time)
    time_mode = SimplePID::TIME_SIM;
  else
    time_mode = SimplePID::TIME_REAL;
  
  linearPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_STANDARD, time_mode);
}

void ThrustController::initAngularPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{  
  SimplePID::TIME_MODE time_mode;
  if (use_sim_time)
    time_mode = SimplePID::TIME_SIM;
  else
    time_mode = SimplePID::TIME_REAL;

  angularPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_CIRCULAR, time_mode);
}

void ThrustController::setTarget
  (double linear_velocity, double angle)
{
  angularPID_->setSetpoint(angle);

  double error_angle = angularPID_->getError();

  // If current heading is too different from desired course
  if (fabs(error_angle) > 0.5*priority_yaw_range_)
  {
    linear_velocity = 0; // Reduce velocity and focus on yawing
  }
  else // If current heading is close enough to desired course
  {
    linear_velocity =    // Scale velocity inversely with angular error
      linear_velocity*(1 - fabs(error_angle)/(0.5*priority_yaw_range_));
  }

  linearPID_ ->setSetpoint(linear_velocity);
}

void ThrustController::setOdometry
  (double linear_velocity, double angle)
{
  linearPID_ ->setObservation(linear_velocity);
  angularPID_->setObservation(angle);
}

double ThrustController::constrainThrust(double thrust)
{
  // Constrain thrust to maximum motor command limit
  if (fabs(thrust) > motor_cmd_limit_)
    thrust = signf(thrust)*motor_cmd_limit_;

  // Scale negative thrust based on negative scale factor
  if (thrust < 0)
    thrust = thrust*neg_scale_factor_;
  
  return thrust;
}

void ThrustController::computeControlSignals(float &thrust_right, float &thrust_left,
  double sim_time)
{
  double lin_ctrl_signal = linearPID_ ->getControlSignal(sim_time);
  double ang_ctrl_signal = angularPID_->getControlSignal(sim_time);

  // Generate thrust commands from PID control signals and constrain
  thrust_right = (float)
    ThrustController::constrainThrust(lin_ctrl_signal + ang_ctrl_signal);        
  thrust_left  = (float)
    ThrustController::constrainThrust(lin_ctrl_signal - ang_ctrl_signal);  
}

}