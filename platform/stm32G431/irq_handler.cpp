#include "adc.h"
#include "gpio.h"
#include "stm32g4xx_hal.h"

#include "dma.h"
#include "usart.h"

#include "foc_runner.hpp"
#include "uart_link.hpp"

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

extern "C" void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

// USART2 TX uses DMA1_Channel1 in this project.
extern "C" void DMA1_Channel1_IRQHandler(void)
{
    if (huart2.hdmatx)
    {
        HAL_DMA_IRQHandler(huart2.hdmatx);
    }
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc && hadc->Instance == ADC1)
    {
        // Handle ADC1 injected conversion complete
        uint16_t adc1_injected_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t adc2_injected_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        platform::current_sense.isr_update_currents(adc1_injected_value, adc2_injected_value);
        platform::motor_control_isr();

        static uint16_t irq_counter = 0;

        if (++irq_counter >= 10000)
        {
            irq_counter = 0;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart2)
    {
        platform::on_uart_rx_byte(platform::g_rx_byte);
        // re-arm RX
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&platform::g_rx_byte, 1);
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart2)
    {
        platform::on_uart_tx_done();
    }
}