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
void ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    {
        remote.getChannel(FlySky::Channel::Left_HORIZONTAL);
        {
            if remote
                .keyPressed(Remote::Key::A) return -1.0f;
            if remote
                .keyPressed(Remote::Key::D) return 1.0f;
        }
    }
    void ControlOperatorInterface::getDrivetrainVerticalTranslation()
    {
        {
            remote.getChannel(FlySky::Channel::LEFT_VERTICAL);
            {
                if remote
                    .keyPressed(Remote::Key::S) return -1.0f;
                if remote
                    .keyPressed(Remote::Key::W) return 1.0f;
            }
        }
        void ControlOperatorInterface::getDrivetrainRotationalTranslation()
        {
            remote.getChannel(FLySky::Channel::WHEEL);
            if remote
                .KeyPressed(Remote::Key::Q) return -MAX_ROTATIONAL_SPEED_RPS;
            if remote
                .KeyPressed(Remote::Key::E) return MAX_ROTATIONAL_SPEED_RPS;
        }
    }

}  // namespace control

}  // namespace control

#endif  // FLY_SKY