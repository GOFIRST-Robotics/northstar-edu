#ifdef TARGET_CHASSIS

#include "chassis_subsystem.hpp"

#include <cmath>

#include <numbers>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
/* Chassis task 2:
    STEP 1: DEFINE THE CONSTRUCTOR

    A constructor is a special function that builds a new object. It must have the
    same name as the class. It will take two arguments:
    1. A POINTER to the shared `tap::Drivers` object. A pointer is like an address
       that tells us where the object lives in memory. We use a pointer (e.g., `tap::Drivers*
   drivers`) to avoid making slow, unnecessary copies.
    2. A CONSTANT REFERENCE to the `ChassisConfig` object. A reference is like a
       nickname for an existing object. We use a reference (e.g., `const ChassisConfig& config`)
       to efficiently access the configuration without copying it. The `const` keyword
       is a promise that we won't change the configuration.

       STEP 2: BEGIN THE MEMBER INITIALIZER LIST

    After the constructor's parameter list, type a colon `:`. This starts the member
    initializer list, which is the best place to set up all your member variables.

    STEP 3: INITIALIZE THE PARENT CLASS

    Because this class inherits from `Subsystem`, the first thing you must do in the
    initializer list is call the parent's constructor. Do this by writing `Subsystem(drivers)`.
    This passes the drivers pointer up to the parent class, which needs it to function.
    Put a comma after it.

    STEP 4: INITIALIZE THE MEMBER VARIABLES

    Now, initialize each member variable, separating each one with a comma.

    - `desiredOutput`: Initialize this using empty curly braces `{}`. This is a
      modern C++ way to zero-initialize a variable, ensuring it starts in a clean state.

    - `pidControllers`: This is an array of PID controllers. Initialize it using
      curly braces `{}`. Inside, create four `modm::Pid<float>` objects. For each one,
      pass in the required constants for Kp, Ki, Kd, max error sum, and max output
      (e.g., `VELOCITY_PID_KP`, `VELOCITY_PID_KI`, etc.).

    - `motors`: This is the array for your four motors. Initialize it using curly braces `{}`.
      Inside, create four `Motor` objects. The constructor for each motor needs:
      1. The `drivers` pointer.
      2. The correct motor ID from the `config` object (e.g., `config.leftFrontId`).
      3. The CAN bus from the `config` object.
      4. A boolean for inversion (e.g., `false`).
      5. A string for its name (e.g., `"LF"` for Left Front).

    STEP 5: CREATE THE CONSTRUCTOR BODY

    After the last item in the initializer list, there is no comma. Now, just add
    an empty pair of curly braces `{}`. Because all the setup work was done in the
    initializer list, the body of the constructor is empty, which is perfectly normal.
*/

// STEP 1-4 HERE
ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers, const ChassisConfig& config)
    : Subsystem(drivers),
      desiredOutput{},
      pidControllers{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)},
      motorArray{
          Motor(drivers, config.leftFrontId, config.canBus, false, "LF"),
          Motor(drivers, config.leftBackId, config.canBus, false, "LB"),
          Motor(drivers, config.rightFrontId, config.canBus, false, "RF"),
          Motor(drivers, config.rightBackId, config.canBus, false, "RB")}
{
}

void ChassisSubsystem::initialize()
{
    for (auto& i : motorArray)
    {
        i.initialize();
    }
}

/* STEP 5: driveBasedOnHeading METHOD

   Our robots use omni wheels for translation, so we have to calculate how fast each wheel
   should spin for the robot to move a certain speed in a certain direction. This method also
   takes in a heading which defines the forward. The math is complicated so here are the
   equations.

   x = forward input
   y = sideways input
   r = rotational input
   h = heading input

   x_local = x * cos(h) + y * sin(h) (this transforms the x velo to the plane of the heading)

   y_local = -x * sin(h) + y * cos(h) (this transforms the y velo to the plane of the heading)

   left_front = (x_local - y_local) / sqrt2 + (r) * DIST_TO_CENTER * sqrt2;

   right_front = (-x_local - y_local) / sqrt2 + (r) * DIST_TO_CENTER * sqrt2;

   right_back = (-x_local + y_local) / sqrt2 + (r) * DIST_TO_CENTER * sqrt2;

   left_back = (x_local + y_local) / sqrt2 + (r) * DIST_TO_CENTER * sqrt2;


   Currently these are in units or meters per second so these need to be converted to rpm. You
   can use the mpsToRpm function to do so but make sure to take a look at how it works. Once
   they are in rpm you need to set the desiredOutput array at the position of the motor id to
   the value. One thing to keep in mind is making sure these values are not to large. Use limit
   val to do so: var _limited = limitVal<float>(var, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
   to make sure you input the correct wheel into the right index of desiredOutput look at the
   MotorId enum to determine what motor is what index. After you set all 4 indexes you are done,
   the motors will be told these values in the refresh method.
*/
// STEP 5 HERE
void ChassisSubsystem::driveBasedOnHeading(
    float forwardV,
    float sidewaysV,
    float rotationalV,
    float heading)
{
    float x_local = forwardV * cos(heading) + sidewaysV * sin(heading);
    float y_local = -forwardV * sin(heading) + sidewaysV * cos(heading);
    float left_front = (x_local - y_local) / sqrt(2.0) + (rotationalV)*DIST_TO_CENTER * sqrt(2.0);
    float right_front = (-x_local - y_local) / sqrt(2.0) + (rotationalV)*DIST_TO_CENTER * sqrt(2.0);
    float right_back = (-x_local + y_local) / sqrt(2.0) + (rotationalV)*DIST_TO_CENTER * sqrt(2.0);
    float left_back = (x_local + y_local) / sqrt(2.0) + (rotationalV)*DIST_TO_CENTER * sqrt(2.0);

    float LFRPM = mpsToRpm(limitVal<float>(left_front, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM));
    float LBRPM = mpsToRpm(limitVal<float>(left_back, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM));
    float RFRPM = mpsToRpm(limitVal<float>(right_front, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM));
    float RBRPM = mpsToRpm(limitVal<float>(right_back, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM));

    desiredOutput[0] = LFRPM;
    desiredOutput[1] = LBRPM;
    desiredOutput[2] = RFRPM;
    desiredOutput[3] = RBRPM;
}

/*
   STEP 6: setVelocityFieldDrive METHOD

   This method uses the previous method to drive field oriented. It also takes in values for
   forward, sideways and rotational velocities. However it dose not take in a heading, it
   calculates one. It dose this by looking at the gyroscope. So all you need to do is get the
   yaw from the gyro like this drivers->bmi088.getYaw(), this in in radians. Once you have this
   just call driveBasedOnHeading with the all the values it needs.

*/
// STEP 6 HERE
void ChassisSubsystem::setVelocityFieldDrive(float forwardV, float sidewaysV, float rotationalV)
{
    float gyroYaw = drivers->bmi088.getYaw();

    driveBasedOnHeading(forwardV, sidewaysV, rotationalV, gyroYaw);
}

/* STEP 7: REFRESH METHOD

Here is the refresh method, go through is and try to understand what is happening.
*/
void ChassisSubsystem::refresh()
{
    auto runPid = [](Pid& pid,
                     tap::algorithms::Ramp& ramp,
                     Motor& motor,
                     float desiredOutput,
                     float increment) {
        ramp.setTarget(desiredOutput);
        ramp.update(increment);
        pid.update(ramp.getValue() - motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI);
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motorArray.size(); ii++)
    {
        runPid(
            pidControllers[ii],
            rampControllers[ii],
            motorArray[ii],
            desiredOutput[ii],
            mpsToRpm(RAMP_UP_RPM_INCREMENT_MPS));
    }
}

}  // namespace src::chassis

#endif
