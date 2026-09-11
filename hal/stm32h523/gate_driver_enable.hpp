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
        GPIO_PinState state = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(pin_a_.port, pin_a_.pin, state);
        if (!single_pin_)
        {
            HAL_GPIO_WritePin(pin_b_.port, pin_b_.pin, state);
            HAL_GPIO_WritePin(pin_c_.port, pin_c_.pin, state);
            HAL_GPIO_WritePin(pin_d_.port, pin_d_.pin, state);
        }
    }

  private:
    PinDef pin_a_{};
    PinDef pin_b_{};
    PinDef pin_c_{};
    PinDef pin_d_{};
    bool single_pin_{true};
};

} // namespace hal
