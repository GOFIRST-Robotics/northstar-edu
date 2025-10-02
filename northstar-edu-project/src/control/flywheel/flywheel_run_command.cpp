#ifdef TARGET_LAUNCHER

#include "flywheel_run_command.hpp"

namespace src::control::flywheel
{
FlywheelRunCommand::FlywheelRunCommand(FlywheelSubsystem* flywheelSubsystem_)
    : flywheelSubsystem(flywheelSubsystem_)
{
    addSubsystemRequirement(flywheelSubsystem);
}

void FlywheelRunCommand::initialize() { flywheelSubsystem->setLaunchSpeedMPS(21.7); }

void FlywheelRunCommand::execute() {}

void FlywheelRunCommand::end(bool interrupted) { flywheelSubsystem->setLaunchSpeedMPS(0); }

}  // namespace src::control::flywheel

#endif  // TARGET_HERO