#include "adc.h"
#include "gpio.h"
#include "stm32f7xx_hal.h"
#include "tim.h"

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

extern "C" void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
    if (htim_encoder->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_IniteStruct = {0};
        GPIO_IniteStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_IniteStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_IniteStruct.Pull = GPIO_NOPULL;
        GPIO_IniteStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_IniteStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_IniteStruct);
    }
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc && hadc->Instance == ADC1)
    {
        // Read Currents.
        uint16_t ia_raw = (uint16_t)(ADC1->JDR1);
        uint16_t ic_raw = (uint16_t)(ADC2->JDR1);

        // Read Vbus and Temp.
        uint16_t vbus_raw = (uint16_t)(ADC1->JDR2);
        uint16_t temp_raw = (uint16_t)(ADC2->JDR2);

        platform::adc_sense.isr_update(ia_raw, ic_raw, vbus_raw, temp_raw);
        platform::motor_control_isr();
    }
}
