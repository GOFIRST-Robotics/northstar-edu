#ifndef VELOCITY_AGITATOR_SUBSYSTEM_HPP_
#define VELOCITY_AGITATOR_SUBSYSTEM_HPP_

#include "tap/architecture/conditional_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"
#include "tap/util_macros.hpp"

#include "velocity_agitator_subsystem_config.hpp"

namespace src
{
class Drivers;
}

namespace src::agitator
{
class VelocityAgitatorSubsystem : public tap::control::setpoint::IntegrableSetpointSubsystem
{
public:
    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = (3591.0f / 187.0f);

    /* Agitator task 1

    STEP 1: DEFINE THE CONSTRUCTOR
    This constructor will need three parameters:
        -A pointer to the shared tap::Drivers object
        -A constant reference to a tap::algorithms::SmoothPidConfig object for the velocity PID
    controller -A constant reference to a VelocityAgitatorSubsystemConfig object for the agitator
    configuration

    STEP 2: SET REFRESHSAFEDISCONNECT
    This method is called when the remote isnt connected. In here you should set the agitator motor
    output to 0 and set the subsystemJamStatus to false. This will make sure the agitator stops
    moving.

    STEP 3: LOOK THROUGH THE REST
    Look through the rest of the class and try to understand what is going on. Also go ahead and
    look at tbe jamChecker Object and see what it does. You will need to use it later.
    */

    // STEP 1 HERE

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        // STEP 2 HERE
    }

    const char* getName() const override { return "velocity agitator"; }

    /// @return The velocity setpoint that some command has requested, in radians / second
    inline float getSetpoint() const override { return velocitySetpoint; }

    /**
     * Sets the velocity setpoint to the specified velocity
     *
     * @param[in] velocity The desired velocity in radians / second.
     */
    void setSetpoint(float velocity) override;

    /// @return The agitator velocity in radians / second.
    inline float getCurrentValue() const override
    {
        return agitatorMotor.getEncoder()->getVelocity();
    }

    /**
     * Meaningless function that nothing uses
     * @return 0
     */
    inline float getJamSetpointTolerance() const override { return 0; }

    /**
     * Attempts to calibrate the agitator at the current position, such that `getPosition` will
     * return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false` otherwise.
     */
    bool calibrateHere() override;

    /**
     * @return `true` if the agitator unjam timer has expired, signaling that the agitator has
     * jammed, `false` otherwise.
     */
    bool isJammed() override { return config.jamLogicEnabled && subsystemJamStatus; }

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    inline void clearJam() override
    {
        subsystemJamStatus = false;
        jamChecker.restart();
    }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been called and the
     * agitator motor is online).
     */
    inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    inline bool isOnline() override { return agitatorMotor.isMotorOnline(); }

    /**
     * Since we don't keep track of the derivative of the velocity (since the velocity is the
     * setpoint), this function will always return 0.
     *
     * @return 0
     */
    inline float getVelocity() override { return 0; }

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     * radians is returned.
     */
    float getCurrentValueIntegral() const override;

private:
    VelocityAgitatorSubsystemConfig config;

    tap::algorithms::SmoothPid velocityPid;

    /// The object that runs jam detection.
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /// Stores the jam state of the subsystem
    bool subsystemJamStatus = false;

    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated = false;

    /// Previous time the velocity controller was called, in milliseconds
    uint32_t prevTime = 0;

    /// The velocity setpoint in radians / second
    float velocitySetpoint = 0;

    /// Runes the velocity PID controller
    void runVelocityPidControl();

    tap::motor::DjiMotor agitatorMotor;
};

}  // namespace src::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM_HPP_