#pragma once

#include <cstdint>
#include "stm32g4xx_hal.h"

namespace pinning {

struct GpioPin {
    GPIO_TypeDef* port;
    uint16_t pin;
};

// ======================
// MCU Pin Definitions
// ======================

// PA5
inline constexpr GpioPin PA5{
    GPIOA,
    GPIO_PIN_5
};

} // namespace pinning
