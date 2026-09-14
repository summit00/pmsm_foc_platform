#pragma once

#include "stm32h5xx_hal.h"
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

// Status and Error LEDs
inline constexpr GpioPin PD3{GPIOD, GPIO_PIN_3};
inline constexpr GpioPin PD5{GPIOD, GPIO_PIN_5};

// DRV830x Gate Driver Control & Status
inline constexpr GpioPin PB10{GPIOB, GPIO_PIN_10};
inline constexpr GpioPin PB12{GPIOB, GPIO_PIN_12};

// SPI Chip Select
inline constexpr GpioPin PB6{GPIOB, GPIO_PIN_6};

// USB Vbus Sensing
inline constexpr GpioPin PA10{GPIOA, GPIO_PIN_10};

} // namespace pinning
