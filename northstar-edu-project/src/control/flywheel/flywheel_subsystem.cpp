#ifdef TARGET_LAUNCHER
#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      velocityPidLeftWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      velocityPidRightWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      leftWheel(drivers, leftMotorId, canBus, true, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0){};

void FlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void FlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);

    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight));
}

float FlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    return launchSpeed / (WHEEL_DIAMETER * M_PI) * 60;
}

void FlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));

    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRampLeft.getValue() - getWheelRPM(&leftWheel));
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));

    velocityPidRightWheel.update(desiredRpmRampRight.getValue() - getWheelRPM(&rightWheel));
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO