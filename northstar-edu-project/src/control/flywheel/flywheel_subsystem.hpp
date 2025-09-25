#ifdef TARGET_LAUNCHER

#ifndef FLYWHEEL_SUBSYSTEM
#define FLYWHEEL_SUBSYSTEM

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
class FlywheelSubsystem : public tap::control::Subsystem
{
public:
    /* Flywheel task 1
    STEP 1: DECLARE THE CONSTRUCTOR
    Name needs to be same as class name, and needs to take in a drivers pointer and a left and right
    motor Id of type tap::motor::MotorId
    */

    // void initialize() override; UNCOMMENT THIS

    /* STEP 2: DECLARE METHODS
    Flywheel subsystem needs a setter for a desired launch speed, and setters for Flywheel speeds.
    Launch speed should be in m/s and flywheel speeds should be in rpm. This means we need a
    method that can convert m/s to rpm. Simple methods like that can be implemented right in the
    .hpp file. We also need to be able to get the current rpm of each motor.

    So here are the methods you need to define:
        set launch speed with a mps.

        get desired launch speed. One for each motor since when you implement the setter it will
        independently set left and right.

        PRIVATE launch speed to flywheel rpm. you can use the WHEEL_DIAMETER constant defined in
        "control/flywheel/flywheel_constants.hpp"

        getters for desired flywheel speed. these can use the stored desired m/s and call the launch
        speed to flywheel rpm method.

        get current flywheel rpm for each motor. use getWheelRPM(const tap::motor::DjiMotor *motor)
        that you will define in step 3.

        PRIVATE get wheel rpm which takes in a motor pointer and returns the rpm. Use this return
        motor->getEncoder()->getVelocity() * 60.0f / M_TWOPI; this can be implemented in the .hpp
    */

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);
        rightWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Flywheels"; }

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED = 25;

    tap::Drivers *drivers;

private:
    /* STEP 3: DEFINE PRIVATE GLOBAL VARIABLES
    we need a pid object for each motor the type is modm::Pid<float>

    floats to store the desired launch speeds.

    two tap::algorithms::Ramp objects to ramp up the flywheels

    a tap::motor::DjiMotor for each motor


    */
    uint32_t prevTime = 0;

    // put PRIVATE methods here
};

}  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_SUBSYSTEM

#endif  // TARGET_HERO