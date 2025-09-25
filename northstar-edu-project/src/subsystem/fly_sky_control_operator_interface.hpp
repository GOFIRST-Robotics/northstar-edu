//#define FLY_SKY
#ifdef FLY_SKY
#ifndef FLY_SKY_CONTROL_OPERATOR_INTERFACE_HPP_
#define FLY_SKY_CONTROL_OPERATOR_INTERFACE_HPP_

// mm tasty imports
#include <tap/algorithms/linear_interpolation_predictor.hpp>
#include <tap/algorithms/ramp.hpp>

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "communication/serial/fly_sky.hpp"

namespace src
{
namespace control
{
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::Drivers *drivers) : remote(drivers->remote) {}

    /* Chassis Task 5: ADD METHODS FOR

        -getDrivetrainHorizontalTranslation
        -getDrivetrainVerticalTranslation
        -getDrivetrainRotationalTranslation
    Here */
    float getDrivetrainHorizontalTranslation();
    float getDrivetrainVerticalTranslation();
    float getDrivetrainRotationalTranslation();

private:
    tap::communication::serial::FlySky &remote;
};
}  // namespace control

}  // namespace src

#endif  // FLY_SKY_CONTROL_OPERATOR_INTERFACE_HPP_
#endif  // FLY_SKY