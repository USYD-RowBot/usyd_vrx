#pragma once

#include <ros/ros.h>
#include <math.h>

namespace usyd_vrx {

class ThrustSM
{
  public:
    /*!
    * Constructor.
    * @param strafe_duration time (secs) to strafe for, in station keeping.
    * @param reconfig_duration time (secs) to allow thruster reconfiguration. 
    */
    ThrustSM(float strafe_duration, float reconfig_duration);

    //! Enum to specify the current state of the thrusting action.
    enum THRUST_STATE {THRUST_TRAVERSE, THRUST_ROTATE, THRUST_STRAFE,
      THRUST_RECONFIG_STRAIGHT, THRUST_RECONFIG_LATERAL};

    /*!
    * Resets the thruster state machine.
    */
    void resetStateMachine();

    /*!
    * Inform state machine about station keeping requests. Crucial in changing
    *   state machine state.
    * @param keep_station whether we have been requested to perform station
    *   keeping or not. 
    */
    void updateState(bool keep_station);

    /*!
    * Checks the strafe and reconfiguration timers and updates state if complete.
    */
    void checkTimers();

    /*!
    * Makes appropriate changes for the given reconfiguration request.
    * @param reconfiguration which type of reconfiguration, either
    *   THRUST_RECONFIG_STRAIGHT or THRUST_RECONFIG_LATERAL
    */
    void reconfigure(THRUST_STATE reconfiguration);

    /*!
    * Marks completion of angular alignment procedure, updates state machine.
    */
    void reportRotationComplete();

    /*!
    * Getter for the current state of the state machine.
    * @return the current state machine state, as an enum.
    */
    THRUST_STATE getState();

  private:
    //! Current state of thrusting action for the object.
    THRUST_STATE state_;

    //! Duration (secs) to perform strafing action.
    float strafe_duration_;

    //! Target time to perform strafing until.
    double strafe_time_target_;

    //! Duration (secs) to perform thruster reconfiguration.
    float reconfig_duration_;

    //! Target time to perform thruster reconfiguration until.
    double reconfig_time_target_;
};

}