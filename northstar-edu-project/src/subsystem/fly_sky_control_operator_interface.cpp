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
float ControlOperatorInterface::getDrivetrainHorizontalTranslation(){

    //potential double speed if somebody uses the channel and the keys at the same time
    float input = remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL);
    return input;
}

float ControlOperatorInterface::getDrivetrainVerticalTranslation(){

    //potential double speed if somebody uses the channel and the keys at the same time
    float input = remote.getChannel(FlySky::Channel::LEFT_VERTICAL);
    return input;
}

float ControlOperatorInterface::getDrivetrainHorizontalTranslation(){

    //potential double speed if somebody uses the channel and the keys at the same time
    float input = remote.getChannel(FlySky::Channel::RIGHT_HORIZONTAL);
    return input;
}

}  // namespace control

}  // namespace src

#endif  // FLY_SKY