#pragma once

#include "stm32f7xx_hal.h"
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
inline constexpr GpioPin PB0{GPIOB, GPIO_PIN_0};

// Powerstage enable pins
inline constexpr GpioPin PD4{GPIOD, GPIO_PIN_4};
inline constexpr GpioPin PD5{GPIOD, GPIO_PIN_5};
inline constexpr GpioPin PD6{GPIOD, GPIO_PIN_6};
inline constexpr GpioPin PD7{GPIOD, GPIO_PIN_7};

} // namespace pinning
