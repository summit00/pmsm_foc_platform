#include "adc.h"
#include "gpio.h"
#include "stm32g4xx_hal.h"

#include "foc_runner.hpp"

extern "C"
{
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

extern "C" void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc && hadc->Instance == ADC1)
    {
        // Read Currents.
        uint16_t ic_raw = (uint16_t)(ADC1->JDR1);
        uint16_t ia_raw = (uint16_t)(ADC2->JDR1);

        // Read Vbus and Temp.
        uint16_t vbus_raw = (uint16_t)(ADC1->JDR2);
        uint16_t temp_raw = (uint16_t)(ADC2->JDR2);

        platform::adc_sense.isr_update(ia_raw, ic_raw, vbus_raw, temp_raw);
        platform::motor_control_isr();
    }
}
