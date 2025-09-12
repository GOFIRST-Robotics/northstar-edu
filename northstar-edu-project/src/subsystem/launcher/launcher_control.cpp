#ifdef TARGET_LAUNCHER

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "drivers_singleton.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// safe disconect
#include "control/safe_disconnect.hpp"

using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::control;
using namespace src::launcher;
using namespace tap::communication::serial;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;

driversFunc drivers = DoNotUse_getDrivers;

namespace launcher_control
{
// flywheel subsystem
FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
ToggleCommandMapping fPressedFlywheels(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));

ToggleCommandMapping leftSwitchUpFlywheels(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

void initializeSubsystems(Drivers *drivers)
{
    flywheel.initialize();
    agitator.initialize();
}

void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&agitator);
}

void setDefaultStandardCommands(Drivers *drivers) {}

void startStandardCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
}

void registerStandardIoMappings(Drivers *drivers) {}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

}  // namespace launcher_control

namespace src::launcher
{
void initSubsystemCommands(src::launcher::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &launcher_control::remoteSafeDisconnectFunction);
    launcher_control::initializeSubsystems(drivers);
    launcher_control::registerStandardSubsystems(drivers);
    launcher_control::setDefaultStandardCommands(drivers);
    launcher_control::startStandardCommands(drivers);
    launcher_control::registerStandardIoMappings(drivers);
}
}  // namespace src::launcher

#endif