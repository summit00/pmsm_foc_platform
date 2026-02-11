#pragma once
#include <cstdint>
#include "interfaces.hpp"
#include "stm32g4xx_hal.h"

//struct GPIO_TypeDef;

namespace hal {

class GpioOutHal final : public app::IDigitalOut {
public:
  GpioOutHal(GPIO_TypeDef* port, uint16_t pin);
  void toggle() override;

private:
  GPIO_TypeDef* port_;
  uint16_t pin_;
};

} // namespace hal
