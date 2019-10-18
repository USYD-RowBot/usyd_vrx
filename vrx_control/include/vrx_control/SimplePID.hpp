#pragma once

#include <chrono>
#include <iostream>

namespace usyd_vrx {

class SimplePID
{
  public:
    //! Enumerator for defining error calculation type.
    enum ERROR_TYPE {ERROR_STANDARD, ERROR_CIRCULAR};

    //! Enumerator for defining timing source for PID control.
    enum TIME_MODE {TIME_REAL, TIME_SIM};

    /*!
    * Constructor.
    * @param Kp the proportional term gain.
    * @param Ki the integral term gain.
    * @param Kd the derivative term gain.
    * @param max_integral the maximum value the error integral can 
    *   accumulate to.
    * @param error takes enum value to specify type of error, either
    *   standard or circular (wrapping error from -PI to PI).
    * @param time takes enum value to specify type of time, either
    *   real time or simulator time.
    */
    SimplePID(float Kp, float Ki, float Kd, float max_integral, 
      ERROR_TYPE error=ERROR_STANDARD,
      TIME_MODE time=TIME_REAL);

    /*!
    * Updates PID controller with a new setpoint.
    * @param setpoint the new target value of the controlled variable.
    */
    void setSetpoint(double setpoint);

    /*!
    * Updates PID controller with a new observation.
    * @param observation the observed value of the controlled variable.
    */
    void setObservation(double observation);

    /*!
    * Calculates control signal based on setpoint, observation and PID gains.
    * @return new control signal; the PID controller output.
    */
    double getControlSignal(double time_now);

    /*!
    * Resets PID controller private variables.
    */
    void resetPID();

    /*!
    * Get the current error calculation.
    * @return the current error between setpoint and observation.
    */
    double getError();

    /*!
    * Get the current setpoint value.
    * @return the setpoint value.
    */
    double getSetpoint();

  private:
    //! Proportional gain.
    float Kp_;

    //! Integral gain.
    float Ki_;

    //! Derivative gain.
    float Kd_;

    //! Previous error term.
    double prev_error_;

    //! Integral of the error term.
    double error_integral_;

    //! Real time at which previous control signal was calculated.
    std::chrono::time_point<std::chrono::high_resolution_clock> time_prev_real_;

    //! Simulator time at which previous control signal was calculated.
    double time_prev_sim_;

    //! Maximum allowed value of integral of error term.
    float max_integral_;

    //! PID setpoint; the target value of the controlled variable.
    double setpoint_;

    //! Observed value of the controlled variable.
    double observation_;

    //! Function pointer to error calculation function.
    double (SimplePID::* error_function_) ();

    //! Function pointer to time calculation function.
    double (SimplePID::* time_function_) (double);

    /*!
    * Gets elapsed real time.
    * @param time_now allows passing custom time source to function.
    * @return time elapsed since previous control signal calculation.
    */
    double getTimeReal(double time_now=0);

    /*!
    * Gets elapsed simulator time.
    * @param time_now current time in seconds from simulator source.
    * @return time elapsed since previous control signal calculation.
    */
    double getTimeSim(double time_now);

    /*!
    * Gets time elapsed since previous control signal calculation.
    * @param time_now allows passing custom time source to function.
    * @return time elapsed since previous control signal calculation.
    */
    double getTimeElapsed(double time_now);

    /*!
    * Updates integral term by accumulating error.
    * @param error the current error between setpoint and observation.
    */
    void accumulate(double error, double dt);

    /*!
    * Calculates error term simply by subtracting observation from setpoint.
    */
    double errorStandard();

    /*!
    * Calculates error term, assuming values are between -PI and PI and wrap.
    */
    double errorCircular();
};

}