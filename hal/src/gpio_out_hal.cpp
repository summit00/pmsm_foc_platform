#include "hal/gpio_out_hal.hpp"
#include "stm32g4xx_hal.h"

namespace hal {

GpioOutHal::GpioOutHal(GPIO_TypeDef* port, uint16_t pin)
: port_(port), pin_(pin) {}

void GpioOutHal::toggle()
{
  HAL_GPIO_TogglePin(port_, pin_);
}

} // namespace hal
