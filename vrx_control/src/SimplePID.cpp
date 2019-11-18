#include "SimplePID.hpp"
#include <math.h>

static int signf(float value)
{
  return (value > 0) - (value < 0);
}

namespace usyd_vrx {

SimplePID::SimplePID(float Kp, float Ki, float Kd, float max_integral, 
  ERROR_TYPE error, TIME_MODE time):

  Kp_(Kp), Ki_(Ki), Kd_(Kd), max_integral_(fabs(max_integral)) 
{
  switch (error) // Set error function type
  {
    case ERROR_STANDARD:
      error_function_ = &SimplePID::errorStandard;  
      break;
    case ERROR_CIRCULAR:
      error_function_ = &SimplePID::errorCircular;
      break;
  }    

  switch (time) // Set time function type
  {
    case TIME_REAL:
      time_function_ = &SimplePID::getTimeReal;  
      break;
    case TIME_SIM:
      time_function_ = &SimplePID::getTimeSim;
      break;
  }

  SimplePID::resetPID();

  setpoint_       = 0;
  observation_    = 0;
}

void SimplePID::resetPID()
{
  prev_error_     = 0;
  error_integral_ = 0;

  time_prev_real_ = std::chrono::high_resolution_clock::now();
  time_prev_sim_  = 0;
}

void SimplePID::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
}

void SimplePID::setObservation(double observation)
{
  observation_ = observation;
}

void SimplePID::accumulate(double error, double dt)
{
  double new_error_integral_ = error_integral_ + error*dt;

  // Constrain integral to have at most "max_integral_" magnitude
  if (fabs(new_error_integral_) > max_integral_)
    new_error_integral_ = signf(new_error_integral_)*max_integral_;

  error_integral_ = new_error_integral_;
}

double SimplePID::getTimeReal(double time_now)
{
  auto time_now_real = std::chrono::high_resolution_clock::now();
  auto time_elapsed  = std::chrono::duration_cast
    <std::chrono::duration<double>>(time_now_real - time_prev_real_);

  // Time (s) elapsed since previous control signal calculation
  double dt = (double)time_elapsed.count(); 
  time_prev_real_ = time_now_real;

  return dt;
}

double SimplePID::getTimeSim(double time_now)
{
  double dt; // Time (s) elapsed since previous control signal calculation

  if (time_prev_sim_ > 0) // If we have a sim time recorded
  {
    dt = time_now - time_prev_sim_;
    time_prev_sim_ = time_now;
  }
  else // If no sim time has been recorded yet
  {
    dt = 0.1; // Treat as reasonable time, e.g. 10Hz
    time_prev_sim_ = time_now;
  }  

  return dt;
}

double SimplePID::getTimeElapsed(double time_now)
{
  // Call time function through pointer
  return (this->*time_function_)(time_now); 
}

double SimplePID::getControlSignal(double time_now)
{
  // Time (s) elapsed since previous control signal calculation
  double dt = SimplePID::getTimeElapsed(time_now);

  // Calculate P,I and D terms
  double error = SimplePID::getError();
  SimplePID::accumulate(error, dt);
  double error_derivative = (error - prev_error_)/dt;

  prev_error_ = error;

  // Simple PID control signal calculation
  return Kp_*error + Ki_*error_integral_ + Kd_*error_derivative;
}

double SimplePID::getError()
{
  return (this->*error_function_)(); // Call error function through pointer
}

double SimplePID::getSetpoint()
{
  return setpoint_;
}

double SimplePID::errorStandard()
{
  return setpoint_ - observation_;
}

double SimplePID::errorCircular()
{
  double error = setpoint_ - observation_;

  // Change error to shortest route around the unit circle from -PI to PI
  if (fabs(error) > M_PI)
    error = -signf(error)*(2*M_PI - fabs(error));
  
  return error;
}

}