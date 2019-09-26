#pragma once

#include <ros/ros.h>

namespace usyd_vrx {

class StationSM
{
  public:
    /*!
    * Constructor.
    */
    StationSM();

    //! Enum to specify the current state of the station keeping action.
    enum STATION_STATE {STATION_NONE, STATION_ALIGN, 
                        STATION_WAIT, STATION_COMPLETE};

    /*!
    * Resets the station keeping state machine. Note that this is the only way
    *   to move to the state STATION_NONE.
    */
    void resetStateMachine();

    /*!
    * Request station keeping action for "duration" seconds.
    * @param duration time in seconds to perform station keeping. Negative means
    *   indefinite wait.
    */
    void startStation(double duration);

    /*!
    * Marks the completion of the alignment procedure, updates state machine.
    */
    void completeAlignment();

    /*!
    * Getter for the current state of the state machine. Checks duration timer.
    *   After state STATION_COMPLETE is reached, resetStateMachine() must be 
    *   called to return the state machine to state STATION_NONE.
    * @return the current state machine state, as an enum.
    */
    STATION_STATE getState();

  private:
    //! Current state of station keeping action for the object.
    STATION_STATE state_;

    /*!
    * Duration (secs) to wait at station after pose alignment. Negative
    *   duration means indefinite wait. 
    */
    double duration_;

    //! Target time to perform station keeping until.
    double time_target_;
};

}