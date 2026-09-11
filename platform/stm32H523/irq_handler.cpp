#include "stm32h5xx_hal.h"

extern "C"
{
    void SysTick_Handler(void)
    {
        HAL_IncTick();
        HAL_SYSTICK_IRQHandler();
    }
}
