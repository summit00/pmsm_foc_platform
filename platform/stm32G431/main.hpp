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
#include "gpio.h"             // CubeMX GPIO init
#include "system_stm32g4xx.h" // CubeMX clock
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
    void MX_ADC1_Init(void);
    void MX_ADC2_Init(void);
}

extern "C"
{
    extern TIM_HandleTypeDef htim1;
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
}

namespace app
{

// This struct holds all your hardware modules
struct MainApp
{
    // HAL tick module
    hal::TickHal tick;

    // LED output
    hal::GpioOutHal led;

    // Heartbeat controller
    app::Heartbeat hb;

    // Constructor sets up everything
    MainApp()
        : led(bsp::status_led().port, bsp::status_led().pin), hb(500) // 500ms heartbeat period
    {
        HAL_Init();
        SystemClock_Config();
        MX_DMA_Init();
        MX_GPIO_Init(); // CubeMX GPIO init
        MX_USART2_UART_Init();
        MX_TIM1_Init(); // CubeMX Timer init
        MX_ADC1_Init(); // CubeMX ADC1 init
        MX_ADC2_Init(); // CubeMX ADC2 init

        // Set IRQ Priorities
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);

        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

        // Step 1: Write slave JSQR first by doing a software-triggered start
        // This arms ADC2's injected sequencer
        HAL_ADCEx_InjectedStart(&hadc2); // arm slave first, no IT
        HAL_ADCEx_InjectedStop(&hadc2);  // stop it again — JSQR is now written

        // Step 2: Now start both properly
        HAL_ADCEx_InjectedStart(&hadc2); // re-arm slave (no IT, no trigger)
        HAL_ADCEx_InjectedStart_IT(&hadc1);

        SET_BIT(ADC2->CR, ADC_CR_JADSTART);

        // Let ADC settle.
        HAL_Delay(10);

        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
        uint32_t sample = arr / 2; // Sample at 50% duty cycle
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample);

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Start PWM for ADC triggering
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        platform::calibrate_current_sense();

        // UART link (commands + telemetry)
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

            // Send telemetry at 500 Hz (every 2ms). Adjust as needed.
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
