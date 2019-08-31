#include "ThrustController.hpp"

static int signf(float value)
{
  return (value > 0) - (value < 0);
}

namespace usyd_vrx {

ThrustController::ThrustController(char thrust_config,
  float priority_yaw_range, float motor_cmd_limit, 
  float lateral_scale_x, float lateral_scale_y,
  float neg_scale_factor, float strafe_thrust, 
  float station_tolerance_ang):

  thrust_config_(thrust_config),
  priority_yaw_range_(fabs(priority_yaw_range)), 
  motor_cmd_limit_(fabs(motor_cmd_limit)),
  lateral_scale_x_(fabs(lateral_scale_x)),
  lateral_scale_y_(fabs(lateral_scale_y)),
  neg_scale_factor_(fabs(neg_scale_factor)),
  strafe_thrust_(fabs(strafe_thrust)),
  station_tolerance_ang_(fabs(station_tolerance_ang))
{
  vessel_yaw_   = 0;
  station_yaw_  = 0;
  strafe_x_     = 0;
  strafe_y_     = 0;
}

ThrustController::~ThrustController()
{
  delete linearPID_;  // Delete objects created with "new" keyword
  delete angularPID_; 
}

void ThrustController::initLinearPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{  
  // Set whether to use simulator time source or real time source for PIDs
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
  // Set whether to use simulator time source or real time source for PIDs
  SimplePID::TIME_MODE time_mode;
  if (use_sim_time)
    time_mode = SimplePID::TIME_SIM;
  else
    time_mode = SimplePID::TIME_REAL;

  angularPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_CIRCULAR, time_mode);
}

void ThrustController::resetPIDs()
{
  angularPID_->resetPID();
  linearPID_->resetPID();
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

  station_yaw_ = angle; // For use when rotational station keeping
}

void ThrustController::setStrafeProportions(double angle)
{
  float rel_angle = M_PI_2 + (float)angle - vessel_yaw_; // Angle relative to vessel

  strafe_x_ = cos(rel_angle); // Set strafe proportions
  strafe_y_ = sin(rel_angle);
}

void ThrustController::setOdometry
  (double linear_velocity, double angle)
{
  vessel_yaw_ = angle; // Record current vessel yaw  
  linearPID_ ->setObservation(linear_velocity);
  angularPID_->setObservation(angle);
}

double ThrustController::constrainThrust(double thrust)
{
  // Constrain thrust to maximum motor command limit
  if (fabs(thrust) > motor_cmd_limit_)
    thrust = signf(thrust)*motor_cmd_limit_;

  // Scale negative thrust based on negative scale factor. Balances speeds.
  if (thrust < 0)
    thrust = thrust*neg_scale_factor_;
  
  return thrust;
}

bool ThrustController::stationAngleHit()
{
  float error = fabs(station_yaw_ - vessel_yaw_);

  std::cout << "error is " << error << ".\n";

  // Change error to shortest route around the unit circle from -PI to PI
  if (error > M_PI)
    error = 2*M_PI - error;

  return (error < station_tolerance_ang_);
}

void ThrustController::getControlSignalTraverse(
  float &thrust_right, float &thrust_left, double sim_time)
{
  double lin_ctrl_signal = linearPID_ ->getControlSignal(sim_time);
  double ang_ctrl_signal = angularPID_->getControlSignal(sim_time);

  // Generate thrust commands from PID control signals and constrain
  thrust_right = (float)
    ThrustController::constrainThrust(lin_ctrl_signal + ang_ctrl_signal);        
  thrust_left  = (float)
    ThrustController::constrainThrust(lin_ctrl_signal - ang_ctrl_signal);  
}

void ThrustController::getControlSignalRotate(float &thrust_right, 
  float &thrust_lat, double sim_time)
{
  double ang_ctrl_signal = angularPID_->getControlSignal(sim_time);

  // Generate thrust commands from PID control signals and constrain. Lateral
  //  thruster is increased by "lateral_scale_x_" to balance the moments
  //  about the boat, since the thrusters are positioned differently.
  thrust_lat   = (float)
    ThrustController::constrainThrust(ang_ctrl_signal*lateral_scale_x_); 
  thrust_right = (float)
    ThrustController::constrainThrust(-ang_ctrl_signal);  
}

void ThrustController::getStrafingThrust(
  float &thrust_right, float &thrust_left, float &thrust_lat)
{
  // Relative y direction strafing, left thruster is at original angle
  thrust_left  = (float)
    ThrustController::constrainThrust(strafe_y_*strafe_thrust_); 

  // Lateral thruster is tuned to balance the moments from the diff thrusters
  //  about the boat, since the thrusters are positioned differently.
  thrust_lat  = (float) ThrustController::constrainThrust(
      strafe_y_*strafe_thrust_*lateral_scale_y_
    - strafe_x_*strafe_thrust_*lateral_scale_x_);  

  // Relative x direction strafing, with right thruster at 90deg angle. 
  thrust_right = (float)
    ThrustController::constrainThrust(-strafe_x_*strafe_thrust_);    
}

}