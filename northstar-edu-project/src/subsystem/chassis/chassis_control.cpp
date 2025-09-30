#ifdef TARGET_CHASSIS

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

// chasis
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// safe disconect
#include "control/safe_disconnect.hpp"

using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::chassis;
using namespace src::control;
using namespace tap::communication::serial;

driversFunc drivers = DoNotUse_getDrivers;

namespace chassis_control
{
/* Chassis task 9

control files are where all subsystems, commands, bindings, and governors are defined and where the
subsystems are initalized and added to the command mapper.

STEP 1: DEFINE A CHASSIS SUBSYSTEM
Create a chassis subsystem by passing in everything that in needed in the constructor. For the
drivers pointer user drivers() to get it.
*/
ChassisSubsystem subsystem{
    drivers(),
    ChassisConfig{
        MotorId::MOTOR2,
        MotorId::MOTOR3,
        MotorId::MOTOR4,
        MotorId::MOTOR1,
        tap::can::CanBus::CAN_BUS1,
        modm::Pid<float>::Parameter(
            VELOCITY_PID_KD,
            VELOCITY_PID_KI,
            VELOCITY_PID_KD,
            VELOCITY_PID_MAX_ERROR_SUM,
            VELOCITY_PID_MAX_OUTPUT)}};
/* STEP 2: DRIVE COMMAND
create a chassis drive command. To get the control operator interface use
drivers()->controlOperatorInterface
*/
src::chassis::ChassisDriveCommand drive_command{&subsystem, &(drivers()->controlOperatorInterface)};
/*
void drive(){
    float x  = drivers()->controlOperatorInterface.getDrivetrainHorizontalTranslation();
    float y  = drivers()->controlOperatorInterface.getDrivetrainVerticalTranslation();
    float r  = drivers()->controlOperatorInterface.getDrivetrainRotationalTranslation();
    subsystem.setVelocityFieldDrive(x, y, r);
}
*/

void initializeSubsystems(Drivers *drivers) { subsystem.initialize(); }

void registerStandardSubsystems(Drivers *drivers)
{
    // STEP 4: REGISTER SUBSYSTEMS
    // pass in chassis subsystem to this method.
    // drivers->commandScheduler.registerSubsystem();
    drivers->commandScheduler.registerSubsystem(&subsystem);
}

void setDefaultStandardCommands(Drivers *drivers)
{
    /* STEP 5: SET DEFAULT COMMANDS
    use .setDefaultCommand from any subsystem object and pass in a pointer to a command that will
    run by default.
    */
    subsystem.setDefaultCommand(&drive_command);
}

void startStandardCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
}

void registerStandardIoMappings(Drivers *drivers) {}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

}  // namespace chassis_control

namespace src::chassis
{
void initSubsystemCommands(src::chassis::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &chassis_control::remoteSafeDisconnectFunction);
    chassis_control::initializeSubsystems(drivers);
    chassis_control::registerStandardSubsystems(drivers);
    chassis_control::setDefaultStandardCommands(drivers);
    chassis_control::startStandardCommands(drivers);
    chassis_control::registerStandardIoMappings(drivers);
}
}  // namespace src::chassis

#endif