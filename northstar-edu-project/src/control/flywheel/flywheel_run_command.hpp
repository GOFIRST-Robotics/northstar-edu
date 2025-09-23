#ifdef TARGET_LAUNCHER

#ifndef FLYWHEEL_RUN_COMMAND
#define FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/flywheel_subsystem.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::control::flywheel
{
class FlywheelRunCommand : public tap::control::Command
{
    /* Flywheel task 3
    STEP 1: DEFINE CONSTRUCTOR, METHODS, AND VARIABLES.
    This command will only use the base Command methods so look at the class for reference.

    The constructor will need a FlywheelSubsystem pointer that gets stored privately.
    Now with the Command class as reference define everything
    */
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND

#endif  // TARGET_HERO