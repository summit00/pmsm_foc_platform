#include "stm32g4xx_hal.h"

extern "C" void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
