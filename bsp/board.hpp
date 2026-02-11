#pragma once

#include <cstdint>
#include "interfaces.hpp"
#include "stm32g4xx_hal.h"

namespace bsp {

struct GpioPin {
    GPIO_TypeDef* port;
    uint16_t pin;
};

// Board “status LED”
// Nucleo-style status LED = PA5
inline GpioPin status_led()
{
    return {GPIOA, GPIO_PIN_5};
}

}
