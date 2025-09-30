#ifdef TARGET_CHASSIS

#pragma once

#include <array>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

#define FIELD

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

namespace src::chassis
{
struct ChassisConfig
{
    tap::motor::MotorId leftFrontId;
    tap::motor::MotorId leftBackId;
    tap::motor::MotorId rightBackId;
    tap::motor::MotorId rightFrontId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

class ChassisSubsystem : public tap::control::Subsystem
{
public:
    enum class MotorId : uint8_t
    {
        LF = 0,
        LB,
        RF,
        RB,
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    static constexpr float MAX_WHEELSPEED_RPM = 9000;

    /* Chassis task 1:
    This is a .hpp file which is called a header file. This is used in c++ to create a blueprint for
    a class that can be passed into other files to that they know what is in the class. It is also
    usefull for organisation and allows you to use methods in any order on the .cpp file.

    STEP 1: DECLARE THE CONSTRUCTOR

    The constructor needs to be named the same as the class name so you can scroll up to see the
    definition and copy that name. Now it needs to take in a tap::Drivers* drivers and a const
    ChassisConfig& config. You will learn what these are latter for now just copy them.
    It should look like this Ex. "ClassName(Thing thing1, Thing thing2);"
    */
    //STEP 1 HERE
    ChassisSubsystem(tap::Drivers* const, ChassisConfig& config);
    /*
    STEP 2: DECLARE METHODS

    Just like the constructor we need to declare the methods.
    These methods need to be declared:
        -setVelocityFeildDrive() which needs variables for forward sideways and rotational
    velocities
        -driveBasedOnHeading() which needs the same as the previous method plus a variable
    for the heading
    */
    //STEP 2 HERE
    void setVelocityFieldDrive(float fwd,float side, float rot);
    void driveBasedOnHeading(float fwd,float side, float rot,float heading);
    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].setDesiredOutput(0);
        }
    }

    const char* getName() const override { return "Chassis"; }

    float getYaw();

private:
    inline float mpsToRpm(float mps)
    {
        static constexpr float GEAR_RATIO = 19.0f;
        static float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }

    /* STEP 3: DECLARE VARIABLES

    Variables should also be defined here if they are going to be global.
    These are the variables we need declared here:
        -desiredOutput array of floats which represents the voltage we want the motor at.

    Here is what desiredOutput should look like
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

        -pidControllers array of Pid objects that the motors use.

        -rampControllers array or tap::algorithms::Ramp objects to limit acceleration on the wheels
    */
    //STEP 3 private HERE
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    std::array<Pid,static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

    std::array<tap::algorithms::Ramp,static_cast<uint8_t>(MotorId::NUM_MOTORS)> rampControllers;


protected:
    /* protected varables are able to be used within the class but not accesable from other classes just like a private var. 
    but any classes that inherit this class can use protected varables
        -motors array of Motor objects.

     */
    //STEP 3 protected HERE
    std::array<Motor,static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;

};  // class ChassisSubsystem
}  // namespace src::chassis

#endif
