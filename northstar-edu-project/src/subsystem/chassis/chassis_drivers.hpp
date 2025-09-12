#ifndef CHASSIS_DRIVERS_HPP_
#define CHASSIS_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

// #include "src/mock/turret_mcb_can_comm_mock.hpp"

#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "subsystem/control_operator_interface.hpp"

#endif

namespace src::chassis
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(), controlOperatorInterface(this) {}

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
#endif
};  // class src::StandardDrivers
}  // namespace src::chassis

#endif  // CHASSIS_DRIVERS_HPP_
