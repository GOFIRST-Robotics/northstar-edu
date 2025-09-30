#ifdef TARGET_CHASSIS

#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "subsystem/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
/* Chassis Task 8:

STEP 1: DEFINE THE CONSTRUCTOR
Make the constuctor for this class. Use an initializer list again to set the member variables.
Inside the constructor body, call addSubsystemRequirement and pass in the chassis pointer.

STEP 2: USE EXECUTE TO DRIVE THE ROBOT

Since driving will be a default command for the robot, the execute method will be called continually
by the command scheduler. Make the execute method. In this method, you will read the joystick values
from the operator interface and pass those into the chassis method setVelocityFieldDrive. You should
also use the limitVal method to clamp the passed in values to -1 and 1 and then scale them to the
max chassis speed variable.

STEP 3: MAKE THE END METHOD STOP THE ROBOT

Make the end method set the
chassis velocity to 0 for all parameters.
*/

// STEP 1 HERE
ChassisDriveCommand::ChassisDriveCommand(
    ChassisSubsystem* subsystem,
    control::ControlOperatorInterface* interface)
    : m_subsystem{subsystem},
      m_interface{interface}
{
    addSubsystemRequirement(subsystem);
}
// STEP 2 HERE
void ChassisDriveCommand::execute()
{
    float x = m_interface->getDrivetrainHorizontalTranslation() * MAX_CHASSIS_SPEED_MPS;
    float y = m_interface->getDrivetrainVerticalTranslation() * MAX_CHASSIS_SPEED_MPS;
    float r = m_interface->getDrivetrainRotationalTranslation() * MAX_CHASSIS_SPEED_MPS;
    m_subsystem->setVelocityFieldDrive(x, y, r);
}

void ChassisDriveCommand::end(bool interrupted) { m_subsystem->setVelocityFieldDrive(0, 0, 0); }
};  // namespace src::chassis

#endif