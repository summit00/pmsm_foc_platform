#include "hal/tick_hal.hpp"
#include "stm32g4xx_hal.h"

namespace hal {

uint32_t TickHal::now_ms() const
{
  return HAL_GetTick();
}

} // namespace hal
