#pragma once
#include <cstdint>
#include "app/interfaces.hpp"
#include "stm32g4xx_hal.h"

//struct GPIO_TypeDef;

namespace bsp {

struct GpioPin {
  GPIO_TypeDef* port;
  uint16_t pin;
};

// Board “status LED”
GpioPin status_led();

} // namespace bsp
