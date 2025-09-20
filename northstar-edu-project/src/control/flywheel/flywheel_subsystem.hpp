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
    FlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::motor::MotorId downMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    mockable float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    mockable float getDesiredLaunchSpeedDown() const { return desiredLaunchSpeedDown; }

    mockable float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    mockable float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    mockable float getDesiredFlywheelSpeedDown() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedDown);
    }

    float getCurrentLeftFlywheelMotorRPM() const { return getWheelRPM(&leftWheel); }

    float getCurrentRightFlywheelMotorRPM() const { return getWheelRPM(&rightWheel); }

    float getCurrentDownFlywheelMotorRPM() const { return getWheelRPM(&downWheel); }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);  // TODO CHANGE
        rightWheel.setDesiredOutput(0);
        downWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Flywheels"; }

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED = 10000;  // TODO

    tap::Drivers *drivers;

private:
    modm::Pid<float> velocityPidLeftWheel;
    modm::Pid<float> velocityPidRightWheel;
    modm::Pid<float> velocityPidDownWheel;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;
    float desiredLaunchSpeedDown;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;
    tap::algorithms::Ramp desiredRpmRampDown;

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
    tap::motor::DjiMotor downWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const;

    float getWheelRPM(const tap::motor::DjiMotor *motor) const
    {
        return motor->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }
};

}  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_SUBSYSTEM

#endif  // TARGET_HERO