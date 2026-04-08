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
        // Read registers directly, bypassing HAL completely
        uint16_t adc1_value = (uint16_t)(ADC1->JDR1);
        uint16_t adc2_value = (uint16_t)(ADC2->JDR1);

        platform::current_sense.isr_update_currents(adc2_value, adc1_value);
        platform::motor_control_isr();
    }
}
