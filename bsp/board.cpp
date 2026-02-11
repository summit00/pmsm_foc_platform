#include "board.hpp"
#include "stm32g4xx_hal.h"

namespace bsp {

GpioPin status_led()
{
  // Nucleo-style status LED = PA5
  return {GPIOA, GPIO_PIN_5};
}

} // namespace bsp
