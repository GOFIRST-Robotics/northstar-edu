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
/* Flywheel task 1:
STEP 1: CREATE A FLYWHEEL SUBSYSTEM
make a flywheel subsystem object using constants from flywheel_constants.hpp
for motor ids and can bus.
*/
// flywheel subsystem
FlywheelSubsystem flywheel_subsystem{drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, CAN_BUS};
// STEP 2: CREATE FLYWHEELRUNCOMMAND
// flywheel commands
FlywheelRunCommand flywheel_run_command{&flywheel_subsystem};

/* STEP 3: MAKE COMMAND MAPPING
Commands can be triggered by remote map states. These are things like a keybind
or a switch on the remote. A very common mapping is the toggle command mapping.
This takes in a drivers pointer, a vector of pointers to commands to be run and a RemoteMapState.
Example of a remote map state for pressing f:
RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F}))
and for left switch up:
RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

Make a ToggleCommandMapping for pressing f and one for fliping the left remote switch up. These
should run the flywheel run command.


*/
// flywheel mappings
ToggleCommandMapping flywheel_mapping_f{
    drivers(),
    std::vector<tap::control::Command *>{&flywheel_run_command},
    RemoteMapState{{tap::communication::serial::Remote::Key::F}}};
ToggleCommandMapping flywheel_mapping_switch{
    drivers(),
    std::vector<tap::control::Command *>{&flywheel_run_command},
    RemoteMapState{Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP}};
// AGITATOR WORK HERE

// agitator subsystem HERE
VelocityAgitatorSubsystem agitator_subsystem{
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG};
// agitator commands (ConstantVelocityAgitatorCommand and UnjamSpokeAgitatorCommand with agitator
// configs in constants)
ConstantVelocityAgitatorCommand constant_agitator_command{
    agitator_subsystem,
    constants::AGITATOR_ROTATE_CONFIG};
UnjamSpokeAgitatorCommand unjam_agitator_command{
    agitator_subsystem,
    constants::AGITATOR_UNJAM_CONFIG};
// make a MoveUnjamIntegralComprisedCommand which takes in both previous commands
MoveUnjamIntegralComprisedCommand move_compromised_command{
    *drivers(),
    agitator_subsystem,
    constant_agitator_command,
    unjam_agitator_command};
// Below are mappings for the commands, uncomment and finish them

HoldRepeatCommandMapping leftMousePressedShoot(
    drivers(),
    {&move_compromised_command},
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

HoldRepeatCommandMapping rightSwitchUpShoot(
    drivers(),
    {&move_compromised_command},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    false);

void initializeSubsystems(Drivers *drivers)
{
    // FLYWHEEL STEP 4: INITIALIZE SUBSYSTEM
    flywheel_subsystem.initialize();
    agitator_subsystem.initialize();
    // Initialize agitator
}

void registerStandardSubsystems(Drivers *drivers)
{
    // FLYWHEEL STEP 5: REGISTAR SUBSYSTEM
    drivers->commandScheduler.registerSubsystem(&flywheel_subsystem);
    // Register agitator
    drivers->commandScheduler.registerSubsystem(&agitator_subsystem);
}

void setDefaultStandardCommands(Drivers *drivers) {}

void startStandardCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
}

void registerStandardIoMappings(Drivers *drivers)
{
    // FLYWHEEL STEP 6: ADD COMMAND MAPPINGS
    // use drivers->commandMapper.addMap() passing in the pointer to the mapping.
    drivers->commandMapper.addMap(&flywheel_mapping_f);
    drivers->commandMapper.addMap(&flywheel_mapping_switch);
    // Add agitator mappings
    drivers->commandMapper.addMap(&leftMousePressedShoot);
    drivers->commandMapper.addMap(&rightSwitchUpShoot);
}

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