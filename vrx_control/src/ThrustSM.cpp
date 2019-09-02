#include "ThrustSM.hpp"

namespace usyd_vrx {

ThrustSM::ThrustSM(float strafe_duration, float reconfig_duration):
  strafe_duration_(fabs(strafe_duration)), 
  reconfig_duration_(fabs(reconfig_duration))
{
  ThrustSM::resetStateMachine();
}

void ThrustSM::resetStateMachine()
{
  strafe_time_target_     = 0;
  reconfig_time_target_   = 0;
  state_                  = THRUST_TRAVERSE;
}

void ThrustSM::checkTimers()
{
  switch (state_) 
  {
    case THRUST_RECONFIG_STRAIGHT:
      if (ros::Time::now().toSec() > reconfig_time_target_)
        state_ = THRUST_TRAVERSE; // Normal hull thruster movement
      break;

    case THRUST_RECONFIG_LATERAL:
      if (ros::Time::now().toSec() > reconfig_time_target_)
        state_ = THRUST_ROTATE; // Begin angular alignment prior to strafe
      break;

    case THRUST_STRAFE:
      if (ros::Time::now().toSec() > strafe_time_target_)
        state_ = THRUST_ROTATE; // Switch back to angular alignment
      break;
  }
}

void ThrustSM::reconfigure(ThrustSM::THRUST_STATE reconfiguration)
{
  if (reconfiguration == THRUST_RECONFIG_STRAIGHT ||
      reconfiguration == THRUST_RECONFIG_LATERAL)
  {
    state_ = reconfiguration; // Signal to begin angling thrusters
    reconfig_time_target_ = ros::Time::now().toSec() + reconfig_duration_;
  }
}

void ThrustSM::updateState(bool keep_station)
{
  switch (state_) 
  {
    case THRUST_TRAVERSE:
      if (keep_station == true)
        ThrustSM::reconfigure(THRUST_RECONFIG_LATERAL);
      break;

    case THRUST_ROTATE:
      if (keep_station == false)
        ThrustSM::reconfigure(THRUST_RECONFIG_STRAIGHT);
      break;

    case THRUST_STRAFE:
      if (keep_station == false)
        ThrustSM::reconfigure(THRUST_RECONFIG_STRAIGHT);
      break;
  }

  ThrustSM::checkTimers();
}

ThrustSM::THRUST_STATE ThrustSM::getState()
{
  ThrustSM::checkTimers();
  return state_;
}

void ThrustSM::reportRotationComplete()
{
  if (state_ == THRUST_ROTATE)
  {
    strafe_time_target_ = ros::Time::now().toSec() + strafe_duration_;
    state_ = THRUST_STRAFE; // Wait for strafe duration to elapse
  }
}

}