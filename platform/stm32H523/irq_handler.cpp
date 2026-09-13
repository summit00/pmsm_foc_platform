#include "stm32h5xx_hal.h"
#include "adc.h"
#include "foc_runner.hpp"

extern "C"
{
    extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
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

extern "C" void ADC1_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

extern "C" void ADC2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc2);
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc && hadc->Instance == ADC1)
    {
        // ADC1 Injected: Rank 1 = Iu (PA0), Rank 2 = Iw (PA2)
        uint16_t iu_raw = static_cast<uint16_t>(ADC1->JDR1);
        uint16_t iw_raw = static_cast<uint16_t>(ADC1->JDR2);

        // ADC2 Injected: Rank 1 = Iv (PA1), Rank 2 = DcBus (PC3)
        uint16_t iv_raw = static_cast<uint16_t>(ADC2->JDR1);
        (void)iv_raw;
        uint16_t vbus_raw = static_cast<uint16_t>(ADC2->JDR2);

        // Pass 2 phase currents (Iu, Iw) and DcBus voltage to ADCSense
        hal::ADCSense::isr_update(iu_raw, iw_raw, vbus_raw, 0);

        // Run 20 kHz FOC motor control loop & capture telemetry
        platform::motor_control_isr();
    }
}
