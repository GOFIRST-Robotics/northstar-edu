#ifdef TARGET_CHASSIS

#include "chassis_subsystem.hpp"

#include <cmath>

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

// Now, write your constructor code below this comment.

void ChassisSubsystem::initialize()
{
    for (auto& i : motors)
    {
        i.initialize();
    }
}

// void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
// {
//     float robotHeading = fmod(drivers->bmi088.getYaw(), 2 * M_PI);
//     driveBasedOnHeading(forward, sideways, rotational, robotHeading);
// }

// void ChassisSubsystem::driveBasedOnHeading(
//     float forward,
//     float sideways,
//     float rotational,
//     float heading)
// {
//     double cos_theta = cos(heading);
//     double sin_theta = sin(heading);
//     double vx_local = forward * cos_theta + sideways * sin_theta;
//     double vy_local = -forward * sin_theta + sideways * cos_theta;
//     double sqrt2 = sqrt(2.0);
//     float LFSpeed = mpsToRpm(
//         (vx_local - vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Front-left
//         wheel
//     float RFSpeed = mpsToRpm(
//         (-vx_local - vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Front-right
//         wheel
//     float RBSpeed = mpsToRpm(
//         (-vx_local + vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Rear-right
//         wheel
//     float LBSpeed = mpsToRpm(
//         (vx_local + vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Rear-left wheel
//     int LF = static_cast<int>(MotorId::LF);
//     int LB = static_cast<int>(MotorId::LB);
//     int RF = static_cast<int>(MotorId::RF);
//     int RB = static_cast<int>(MotorId::RB);
//     desiredOutput[LF] = limitVal<float>(LFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
//     desiredOutput[LB] = limitVal<float>(LBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
//     desiredOutput[RF] = limitVal<float>(RFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
//     desiredOutput[RB] = limitVal<float>(RBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
// }

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

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(
            pidControllers[ii],
            rampControllers[ii],
            motors[ii],
            desiredOutput[ii],
            mpsToRpm(RAMP_UP_RPM_INCREMENT_MPS));
    }
}
}  // namespace src::chassis

#endif
