#pragma once
#include "bsp.hpp"
#include "current_adc.hpp"
#include "dwt_cycle_counter.hpp"
#include "foc_runner.hpp"
#include "gpio.h"
#include "gpio_out.hpp"
#include "heartbeat.hpp"
#include "stm32g4xx_hal.h"
#include "tick.hpp"

// UART commands + telemetry
#include "uart_link.hpp"

extern "C"
{
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "system_stm32g4xx.h"
#include "tim.h"
#include "usart.h"
}

extern "C"
{
    void SystemClock_Config(void);
    void MX_GPIO_Init(void);
    void MX_DMA_Init(void);
    void MX_USART2_UART_Init(void);
    void MX_TIM1_Init(void);
    void MX_TIM2_Init(void);
    void MX_ADC1_Init(void);
    void MX_ADC2_Init(void);
}

extern "C"
{
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim2;
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
}

namespace app
{

struct MainApp
{
    hal::TickHal tick;
    hal::GpioOutHal led;
    app::Heartbeat hb;

    MainApp() : led(bsp::status_led().port, bsp::status_led().pin), hb(500)
    {
        HAL_Init();
        SystemClock_Config();
        MX_DMA_Init();
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM1_Init();
        MX_TIM2_Init();
        MX_ADC1_Init();
        MX_ADC2_Init();

        // Set IRQ Priorities
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);

        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
        uint32_t sample = arr - 10;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample);
        HAL_ADCEx_InjectedStart(&hadc2);
        HAL_ADCEx_InjectedStart_IT(&hadc1);

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        platform::init_encoder();

        platform::calibrate_current_sense();

        platform::uart_init();

        hal::DwtCycleCounter::enable();

        // Initialize heartbeat
        // hb.start(tick);
    }

    // Run the main loop
    void loop()
    {
        static uint32_t last_telem_ms = 0;
        while (true)
        {
            platform::check_for_rx_data();
            platform::process_line();

            // Send telemetry at 500 Hz (every 2ms).
            uint32_t now = HAL_GetTick();
            if ((now - last_telem_ms) >= 2)
            {
                last_telem_ms = now;
                platform::telemetry_try_send();
            }

            // Optional: heartbeat LED in background
            // hb.update(tick, led);
        }
    }
};
} // namespace app
