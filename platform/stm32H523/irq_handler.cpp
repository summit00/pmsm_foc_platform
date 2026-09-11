#include "stm32h5xx_hal.h"

extern "C"
{
    extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
}

extern "C" void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
