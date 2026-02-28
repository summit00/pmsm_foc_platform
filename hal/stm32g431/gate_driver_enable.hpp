#pragma once
#include "interfaces.hpp"
#include "stm32g4xx_hal.h"

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

    GateDriverEnable(PinDef a, PinDef b, PinDef c, PinDef d)
        : pin_a(a), pin_b(b), pin_c(c), pin_d(d)
    {
    }

    void set_enable(bool enabled) override
    {
        GPIO_PinState state = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(pin_a.port, pin_a.pin, state);
        HAL_GPIO_WritePin(pin_b.port, pin_b.pin, state);
        HAL_GPIO_WritePin(pin_c.port, pin_c.pin, state);
        HAL_GPIO_WritePin(pin_d.port, pin_d.pin, state);
    }

  private:
    PinDef pin_a;
    PinDef pin_b;
    PinDef pin_c;
    PinDef pin_d;
};

} // namespace hal