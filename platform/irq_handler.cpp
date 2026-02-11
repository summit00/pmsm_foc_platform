#include "stm32g4xx_hal.h"

extern "C" void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

// extern "C" void EXTI15_10_IRQHandler(void)
// {
//   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
// }
