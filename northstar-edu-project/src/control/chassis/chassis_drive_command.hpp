#pragma once

#include "tap/control/command.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::chassis
{
class ChassisSubsystem;

class ChassisDriveCommand : public tap::control::Command
{
public:
    /* Chassis Task 7:
    STEP 1: DECLARE THE CONSTRUCTOR SIGNATURE
    The constuctor for this class will need to take in two parameters:
        -A pointer to the ChassisSubsystem object. This will be used to control the chassis.
        -A pointer to the ControlOperatorInterface object. This will be used to get input from
         the user.
    We will also need to make member variables to store these pointers. Declare them in the private
    section of the class.
    STEP 2: MAKE A MAX RPM VARIABLE Make a static constexpr float variable
    called MAX_CHASSIS_SPEED_MPS and set it to 4.0f.
    */

    // STEP 1 HERE
    // STEP 2 HERE

    ChassisDriveCommand(ChassisSubsystem* chassis, control::ControlOperatorInterface* coi);

    int src::chassis::ChassisDriveCommand::Max_Chassis_Speed_MPS = 4.0f;  // declaration
    static constexpr float MAX_CHASSIS_SPEED_MPS = 4.0f;

    const char* getName() const override { return "Chassis tank drive"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    // the * is a pointer, im gonna forget that's what that means
    // STEP 1 HERE - Make member variables for the pointers.

    ChassisSubsystem* chassisSubsystem;

    control::ControlOperatorInterface* controlOperatorInterface;
};
}  // namespace src::chassis