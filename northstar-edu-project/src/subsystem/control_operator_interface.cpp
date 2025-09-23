//#define FLY_SKY
#ifndef FLY_SKY

/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "subsystem/control_operator_interface.hpp"

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
/*
Chassis Task 4:
STEP 1: CREATE METHODS MADE IN THE HEADER FILE

This step is pretty straightforward. You need to define the methods you made in the header.

STEP 2: GET THE VALUES FROM THE REMOTE OBJECT

Make the methods return the values from the remote object directional channels.
We want to make these methods also use keyboard key inputs if they are available.

An example of directional chanel input is remote.getChannel(Remote::Channel::LEFT_VERTICAL)
An example of key input is remote.keyPressed(Remote::Key::S)
Take note that the values for key presses are 0 for not pressed and 1 for pressed. Vertical and
horizontal channels are percent output with 1.0f being 100% and -1.0f being 100% in the opposite
direction. For rotational output, the method needs to return a radians per second. A good way to
implement this is to multiply the output from the controller by a max speed

After you are done with this step, you should repeat task 3 and 4 for the fly sky remote
operator interface.
*/

float ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    float output = 0.0f;

    if (remote.keyPressed(Remote::Key::D))
    {
        output += 1.0f;
    }
    else if (remote.keyPressed(Remote::Key::A))
    {
        output -= 1.0f;
    }

    output += remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    return limitVal<float>(output, -1.0f, 1.0f);
}

float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    float output = 0.0f;

    if (remote.keyPressed(Remote::Key::W))
    {
        output += 1.0f;
    }
    else if (remote.keyPressed(Remote::Key::S))
    {
        output -= 1.0f;
    }

    output += remote.getChannel(Remote::Channel::LEFT_VERTICAL);
    return limitVal<float>(output, -1.0f, 1.0f);
}

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    return remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * PI;
}

}  // namespace control

}  // namespace src

#endif  // FLY_SKY