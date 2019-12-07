#include "ThrustController.hpp"

static int signf(float value)
{
  return (value > 0) - (value < 0);
}

namespace usyd_vrx {

ThrustController::ThrustController(char thrust_config,
  float priority_yaw_range, float motor_cmd_limit, 
  float lateral_scale_x, float lateral_scale_y,
  float neg_scale_factor, float station_tolerance_ang):

  thrust_config_(thrust_config),
  priority_yaw_range_(fabs(priority_yaw_range)), 
  motor_cmd_limit_(fabs(motor_cmd_limit)),
  lateral_scale_x_(fabs(lateral_scale_x)),
  lateral_scale_y_(fabs(lateral_scale_y)),
  neg_scale_factor_(fabs(neg_scale_factor)),
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
  delete stationXPID_; 
  delete stationYPID_; 
}

SimplePID::TIME_MODE ThrustController::getTimeMode(bool use_sim_time)
{
  // Set whether to use simulator time source or real time source for PIDs
  SimplePID::TIME_MODE time_mode;
  if (use_sim_time)
    time_mode = SimplePID::TIME_SIM;
  else
    time_mode = SimplePID::TIME_REAL;
  return time_mode;
}

void ThrustController::initLinearPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{  
  linearPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_STANDARD, ThrustController::getTimeMode(use_sim_time));
}

void ThrustController::initAngularPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{  
  angularPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_CIRCULAR, ThrustController::getTimeMode(use_sim_time));
}

void ThrustController::initStationPID
  (float Kp, float Ki, float Kd, float max_integral, bool use_sim_time)
{    
  stationXPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_STANDARD, ThrustController::getTimeMode(use_sim_time));

  stationYPID_ = new usyd_vrx::SimplePID(Kp, Ki, Kd, max_integral, 
    SimplePID::ERROR_STANDARD, ThrustController::getTimeMode(use_sim_time));
}

void ThrustController::resetPIDs()
{
  angularPID_->resetPID();
  linearPID_->resetPID();
  stationXPID_->resetPID();
  stationYPID_->resetPID();
}

void ThrustController::setTarget
  (double linear_velocity, double angle)
{
  angularPID_->setSetpoint(angle);
  // Don't need to set target for stationPID because setpoint is always 0 distance.

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

void ThrustController::setStationTarget(double E_x, double E_y)
{
  double error_x = -(E_x*cos(vessel_yaw_) + E_y*sin(vessel_yaw_));
  double error_y = E_y*cos(vessel_yaw_) - E_x*sin(vessel_yaw_);

  /*std::cout << "\nstation yaw is " << station_yaw_ << ", vessel yaw is " << vessel_yaw_ << ".\n";
  std::cout << "E_x is " << E_x << ", E_y is " << E_y << ".\n";
  std::cout << "error_x is " << error_x << ", error_y is " << error_y << ".\n";*/

  stationXPID_->setObservation(error_y); // Switch x and y in controller
  stationYPID_->setObservation(error_x - 0.82);
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

  // Change error to shortest route around the unit circle from -PI to PI
  if (error > M_PI)
    error = 2*M_PI - error;

  return (error < station_tolerance_ang_);
}

void ThrustController::getControlSignalTraverse(
  float &thrust_right, float &thrust_left, float &thrust_lat, double sim_time)
{
  double lin_ctrl_signal = linearPID_ ->getControlSignal(sim_time);
  double ang_ctrl_signal = angularPID_->getControlSignal(sim_time);

  // Use lateral thruster for turning during traversal
  thrust_lat   = (float) 
    ThrustController::constrainThrust(lateral_scale_y_*ang_ctrl_signal);

  // Generate thrust commands from PID control signals and constrain
  thrust_right = (float)
    ThrustController::constrainThrust(lin_ctrl_signal + ang_ctrl_signal);
  thrust_left  = (float)
    ThrustController::constrainThrust(lin_ctrl_signal - ang_ctrl_signal); 
}

void ThrustController::getControlSignalStation(
  float &thrust_right, float &thrust_left, float &thrust_lat, double sim_time)
{
  float ang_ctrl_signal = angularPID_->getControlSignal(sim_time);

  float strafe_x_signal = stationXPID_->getControlSignal(sim_time);
  float strafe_y_signal = stationYPID_->getControlSignal(sim_time);

  //std::cout << "strafe_x_signal is " << strafe_x_signal << ", strafe_y_signal is " << strafe_y_signal << ".\n";

  /*// Lateral thruster is increased by "lateral_scale_x_" to balance the 
  //  moments about the boat, since the thrusters are positioned differently.
  thrust_lat   = (float) ThrustController::constrainThrust(
    -strafe_thrust*strafe_x_*lateral_scale_x_ // Strafe component
    + ang_ctrl_signal*lateral_scale_x_ );     // Rotation component

  // Rear thrusters at 45 degrees thrust oppositely, cancelling forward/back
  //  thrust and resulting in purely lateral thrust. Need to tune
  //  "neg_scale_factor" to achieve the purely lateral thrust.
  thrust_right = (float) ThrustController::constrainThrust(
    strafe_thrust*(strafe_y_ - strafe_x_) // Strafe component
    - ang_ctrl_signal);                   // Rotation component

  thrust_left = (float) ThrustController::constrainThrust(
    strafe_thrust*(strafe_y_ + strafe_x_) // Strafe component
    + ang_ctrl_signal);                   // Rotation component*/

  thrust_lat   = (float) ThrustController::constrainThrust(
    - strafe_x_signal*lateral_scale_x_    // Strafe component
    + ang_ctrl_signal*lateral_scale_x_ ); // Rotation component

  thrust_right = (float) ThrustController::constrainThrust(
    strafe_y_signal - strafe_x_signal // Strafe component
    - ang_ctrl_signal);               // Rotation component

  thrust_left = (float) ThrustController::constrainThrust(
    strafe_y_signal + strafe_x_signal // Strafe component
    + ang_ctrl_signal);               // Rotation component
}

}