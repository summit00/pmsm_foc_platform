#pragma once
#include "interfaces.hpp"
#include "stm32h5xx_hal.h"

namespace hal
{

class GateDriverEnable final : public app::IEnableOutput
{
  public:
    struct PinDef
    {
        GPIO_TypeDef* port;
        uint16_t pin;
    };

    explicit GateDriverEnable(PinDef enable_pin)
        : pin_a_(enable_pin), single_pin_(true)
    {
    }

    GateDriverEnable(PinDef a, PinDef b, PinDef c, PinDef d)
        : pin_a_(a), pin_b_(b), pin_c_(c), pin_d_(d), single_pin_(false)
    {
    }

    void set_enable(bool enabled) override
    {
        (void)enabled;
        // The DRV8353 Smart Gate Driver ENABLE pin (PB10) must remain HIGH continuously
        // to prevent resetting SPI registers (3-PWM mode, CSA gain) and dropping CSAs into sleep mode.
        // Inverter stage enabling/disabling is handled by PWM duty modulation in Inverter.
    }

  private:
    PinDef pin_a_{};
    PinDef pin_b_{};
    PinDef pin_c_{};
    PinDef pin_d_{};
    bool single_pin_{true};
};

} // namespace hal
