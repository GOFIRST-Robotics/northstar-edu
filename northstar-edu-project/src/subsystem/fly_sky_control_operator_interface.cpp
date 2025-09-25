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
float ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    return limitVal<float>(remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL), -1, 1);
}

float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    return limitVal<float>(remote.getChannel(FlySky::Channel::LEFT_VERTICAL), -1, 1);
}

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    return limitVal<float>(remote.getChannel(FlySky::Channel::WHEEL_A) * M_PI, -M_PI, M_PI);
}

}  // namespace control

}  // namespace src

#endif  // FLY_SKY