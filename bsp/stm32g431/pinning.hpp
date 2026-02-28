#pragma once

#include "stm32g4xx_hal.h"
#include <cstdint>

namespace pinning
{

struct GpioPin
{
    GPIO_TypeDef* port;
    uint16_t pin;
};

// ======================
// MCU Pin Definitions
// ======================

// Nucleo LED Pin
inline constexpr GpioPin PA5{GPIOA, GPIO_PIN_5};

// Powerstage enable pins
inline constexpr GpioPin PC10{GPIOC, GPIO_PIN_10};
inline constexpr GpioPin PC11{GPIOC, GPIO_PIN_11};
inline constexpr GpioPin PC12{GPIOC, GPIO_PIN_12};

} // namespace pinning
