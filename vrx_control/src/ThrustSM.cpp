#include "ThrustSM.hpp"

namespace usyd_vrx {

ThrustSM::ThrustSM(float reconfig_duration):
  reconfig_duration_(fabs(reconfig_duration))
{
  ThrustSM::resetStateMachine();
}

void ThrustSM::resetStateMachine()
{
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
        state_ = THRUST_STATION; // Begin station keeping
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

    case THRUST_STATION:
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

}