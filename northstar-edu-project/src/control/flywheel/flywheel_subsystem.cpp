#ifdef TARGET_LAUNCHER
#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
/* Flywheel task 2
STEP 1 DEFINE THE CONSTRUCTOR AND MAKE THE INITIALIZER LIST.

A flywheel subsystem needs to be passed a driver pointer a left and right MotorId, and a can bus.
Now start the initializer list by constructing the parent class Subsystem. Add pid objects for left
and right wheels with the perarmiters defined in launcher_flywheel_constants.hpp. Add motor objects
for each wheel. set desired launch speed left and right to 0 as well as desired rpm.
*/

FlywheelSubsystem::FlywheelSubsystem(tap::Drivers *drivers)
    : Subsystem(drivers),
      drivers(drivers),
      motor_Left(tap::motor::DjiMotor(
          drivers,
          flywheel::LEFT_MOTOR_ID,
          flywheel::CAN_BUS,
          false,
          "Left Flywheel Motor")),
      motor_Right(tap::motor::DjiMotor(
          drivers,
          flywheel::RIGHT_MOTOR_ID,
          flywheel::CAN_BUS,
          false,
          "Right Flywheel Motor"))
{
}

/* STEP 2 CREATE METHODS
initialize:
should call the .initialize of the motors and store the prevTime to be used for pid calculations.
Get time like this tap::arch::clock::getTimeMilliseconds();
*/

void FlywheelSubsystem::initialize()
{
    motor_Left.initialize();
    motor_Right.initialize();

    prevTime = tap::arch::clock::getTimeMilliseconds();
}

/*
set desired launch speed:
needs to set the desired launch speed variable (m/s) for left and right while limiting it from 0 to
MAX_DESIRED_LAUNCH_SPEED. Also needs to set the target of the ramp objects to the rpm equivalent of
the m/s from the launch speed. Use the launch speed to flywheel rpm method you made.
*/

void FlywheelSubsystem::setLaunchSpeedMPS(float desiredSpeed)
{
    float clampedSpeed = limitVal<float>(desiredSpeed, 0, MAX_DESIRED_LAUNCH_SPEED);

    desiredLaunchSpeed_Left = clampedSpeed;
    desiredLaunchSpeed_Right = clampedSpeed;

    float desiredWheelRPM = launchSpeedToFlywheelRPM(clampedSpeed);
    flywheelRamp_Left.setTarget(desiredWheelRPM);
    flywheelRamp_Right.setTarget(desiredWheelRPM);
}

/*
refresh:
this will run every main loop and it handles the pids and telling motors what to do and ramping the
speed. So in order to ramp the speed smooth we need to get the current time and compare it to the
last time, which means you need to find the current time right away. Use
tap::arch::clock::getTimeMilliseconds(); and store it as a uint32_t. Now if that time and the last
time are equal we can return out of the method.

To make the flywheels ramp up you need to call .update on the ramps with this:
"FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime)" passed in.

thats all that prevTime is used for so you can set prevTime = to currTime

Then call .update on the pids and pass in the error which is the current value of the ramp - the rpm
of the motor. So that would be .getValue on the ramp and the getWheelRPM method you made with the
motor passed in. Do that for each pid.

after that you need to set the output of the motors to the pid output. Use the .setDesiredOutput
method from the motor and pass in the pid.getValue().
Do this for both motors.
*/

void FlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (prevTime == currTime)
    {
        return;
    }
    uint32_t deltaTime = currTime - prevTime;
    prevTime = currTime;

    flywheelRamp_Left.update(FRICTION_WHEEL_RAMP_SPEED * deltaTime);
    flywheelRamp_Right.update(FRICTION_WHEEL_RAMP_SPEED * deltaTime);

    motorPID_Left.update(flywheelRamp_Left.getValue() - getCurrentFlywheelRPM_Left());
    motorPID_Right.update(flywheelRamp_Right.getValue() - getCurrentFlywheelRPM_Right());

    motor_Left.setDesiredOutput(motorPID_Left.getValue());
    motor_Right.setDesiredOutput(motorPID_Right.getValue());
}

}  // namespace src::control::flywheel

#endif  // TARGET_HERO