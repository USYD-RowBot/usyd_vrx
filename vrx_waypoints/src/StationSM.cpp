#include "StationSM.hpp"

namespace usyd_vrx {

StationSM::StationSM()
{
  StationSM::resetStateMachine();
}

void StationSM::resetStateMachine()
{
  time_target_ = 0;
  duration_    = 0;
  state_       = STATION_NONE;
}

StationSM::STATION_STATE StationSM::getState()
{
  // If we are currently waiting for the timer to elapse, check it.
  if (state_ == STATION_WAIT)
  {
    // If timer elapsed, and not indefinite duration
    if (ros::Time::now().toSec() > time_target_ && duration_ >= 0) 
      state_ = STATION_COMPLETE;
  }

  return state_;
}

void StationSM::startStation(double duration)
{
  if (duration < 0)
    ROS_INFO_STREAM("Negative duration specified. "
      << "Will wait at station indefinitely after pose alignment.");

  duration_ = duration;
  state_    = STATION_ALIGN; // Begin aligning pose
}

void StationSM::completeAlignment()
{
  if (state_ == STATION_ALIGN)
  {
    time_target_ = ros::Time::now().toSec() + duration_;
    state_ = STATION_WAIT; // Wait for duration to elapse
  }
}

}
