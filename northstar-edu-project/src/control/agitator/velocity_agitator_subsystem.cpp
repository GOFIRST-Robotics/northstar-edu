#ifdef TARGET_LAUNCHER
<<<<<<< HEAD
=======

>>>>>>> main
#include "velocity_agitator_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

using namespace tap::motor;

namespace src::agitator
{
/* Agitator Task 2
STEP 1: DEFINE THE CONSTRUCTOR
Use the constuctor you defined in the .hpp file to create the constructor for the class. You will
need to use a member initializer list to initialize the parent class and all the member variables.
STEP 2: EXAMINE THE INITIALIZE AND REFRESH METHODS
Look at the initialize and refresh methods. Understand whats going on, otherwise ask if you cant.
STEP 3: FINISH THE CALIBRATE METHOD
This method should check if the motor is online, if it is not return false. If it is online
calibrate the motor by resetting the motor encoder value. Then set the agitatorIsCalibrated to true,
set the velocitySetpoint to 0 and clear the jam status. Finally return true.
STEP 4: FINISH THE GETCURRENTVALUEINTEGRAL METHOD
This method should return the unwrapped position of the motor in radians. Do this by getting the
ecoder object and reading its position. If the agitator is not calibrated, it should return 0.
STEP 5: LOOK AT THE LAST TWO METHODS
These methods are used to run the velocity PID and set the velocity setpoint. Look through them
and understand what is going on. If you have questions ask.

*/
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(
    // STEP 1 HERE
    )
    :

{
    assert(config.jammingVelocityDifference >= 0);
}

void VelocityAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void VelocityAgitatorSubsystem::refresh()
{
    if (!isOnline())
    {
        agitatorIsCalibrated = false;
    }

    if (!agitatorIsCalibrated)
    {
        if (!calibrateHere())
        {
            return;
        }
    }

    runVelocityPidControl();

    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

bool VelocityAgitatorSubsystem::calibrateHere()
{
    // STEP 3 HERE
}

float VelocityAgitatorSubsystem::getCurrentValueIntegral() const
{
    // STEP 4 HERE
}

void VelocityAgitatorSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = velocitySetpoint - getCurrentValue();

    velocityPid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(
        velocityPid.getOutput() + velocitySetpoint * config.velocityPIDFeedForwardGain);
}

void VelocityAgitatorSubsystem::setSetpoint(float velocity)
{
    if (agitatorMotor.isMotorOnline())
    {
        velocitySetpoint = velocity;
    }
}
}  // namespace src::agitator

#endif