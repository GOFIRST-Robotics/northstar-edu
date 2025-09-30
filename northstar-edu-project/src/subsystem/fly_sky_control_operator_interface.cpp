//#define FLY_SKY
#ifdef FLY_SKY

#include "subsystem/fly_sky_control_operator_interface.hpp"

#include <random>

#include <tap/architecture/clock.hpp>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace src
{
namespace control
{
/* Chassis task 6:
For the translation methods here, dont worry about keyboard input. An example of accessing a channel
would be remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL)
*/

ControlOperatorInterface::ControlOperatorInterface(tap::Drivers *drivers) : remote(drivers->remote)
{
}

float ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    return 3 * (remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL));
}

float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    return 3 * (remote.getChannel(FlySky::Channel::LEFT_VERTICAL));
}

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    return 3 * (remote.getChannel(FlySky::Channel::RIGHT_HORIZONTAL));
}

}  // namespace control

}  // namespace src

#endif  // FLY_SKY