#ifndef LAUNCHER_FLYWHEEL_CONSTANTS_HPP_
#define LAUNCHER_FLYWHEEL_CONSTANTS_HPP_

#include <optional>

#include <modm/container/pair.hpp>

#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 4.0f;

static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId DOWN_MOTOR_ID = tap::motor::MOTOR1;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr float FLYWHEEL_PID_KP = 30.0f;
static constexpr float FLYWHEEL_PID_KI = 0.0f;
static constexpr float FLYWHEEL_PID_KD = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_OUTPUT = 25'000.0f;

static constexpr float WHEEL_DIAMETER = .075;  // needs to be measured

};  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_CONSTANTS_HPP_