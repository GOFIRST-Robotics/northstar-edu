#ifdef TARGET_LAUNCHER

#include "flywheel_run_command.hpp"

namespace src::control::flywheel
{
/* Flywheel task 4
STEP 1: ADD CONSTRUCTOR INIT LIST AND METHODS:
good luck :)
*/

FlywheelRunCommand::FlywheelRunCommand(FlywheelSubsystem* subsystem) : subsystem_{subsystem}
{
    addSubsystemRequirement(subsystem_);
}

const char* FlywheelRunCommand::getName() const { return "FlywheelRunCommand"; }

void FlywheelRunCommand::end(bool interrupted) { subsystem_->setLaunchSpeed(0); }
void FlywheelRunCommand::initialize() { subsystem_->setLaunchSpeed(10); }

}  // namespace src::control::flywheel

#endif  // TARGET_HERO