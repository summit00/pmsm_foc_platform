#include "adc.h"
#include "gpio.h"
#include "stm32g4xx_hal.h"

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
        // Handle ADC1 injected conversion complete
        uint16_t adc1_injected_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t adc2_injected_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        (void)adc1_injected_value; // Use or store this value as needed
        (void)adc2_injected_value; // Use or store this value as needed

        static uint16_t irq_counter = 0;

        if (++irq_counter >= 10000)
        {
            irq_counter = 0;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}