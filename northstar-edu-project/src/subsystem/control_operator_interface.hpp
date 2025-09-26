//#define FLY_SKY
#ifdef FLY_SKY
#include "subsystem/fly_sky_control_operator_interface.hpp"
#else
/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

// mm tasty imports
#include <tap/algorithms/linear_interpolation_predictor.hpp>
#include <tap/algorithms/ramp.hpp>

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

namespace src
{
namespace control
{
/* Chassis Task 3:
STEP 1: DECLARE METHODS FOR DIRECT TRANSLATION INPUT

In the public section, make three methods that return a float and take no parameters:
    -getDrivetrainHorizontalTranslation
    -getDrivetrainVerticalTranslation
    -getDrivetrainRotationalTranslation
Take note of the remote object that is a member variable of this class. You will be using this in
the next task.
*/
void getDrivetrainHorizontalTranslation();
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::Drivers *drivers) : remote(drivers->remote) {}

private:
    tap::communication::serial::Remote &remote;
};

}  // namespace control

}  // namespace src

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_

#endif  // FLY_SKY