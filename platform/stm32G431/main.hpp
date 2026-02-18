#pragma once
#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "heartbeat.hpp"
#include "tick_hal.hpp"
#include "gpio_out_hal.hpp"
#include "bsp.hpp"

extern "C" {
#include "system_stm32g4xx.h" // CubeMX clock
#include "gpio.h"             // CubeMX GPIO init
#include "adc.h"
#include "tim.h"
}

extern "C" {
    void SystemClock_Config(void);
    void MX_GPIO_Init(void);
    void MX_TIM1_Init(void);
    void MX_ADC1_Init(void);
    void MX_ADC2_Init(void);
}

extern "C" {
    extern TIM_HandleTypeDef htim1;
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
}

namespace app {

// This struct holds all your hardware modules
struct MainApp {
    // HAL tick module
    hal::TickHal tick;

    // LED output
    hal::GpioOutHal led;

    // Heartbeat controller
    app::Heartbeat hb;

    // Constructor sets up everything
    MainApp()
        : led(bsp::status_led().port, bsp::status_led().pin),
          hb(500)  // 500ms heartbeat period
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init(); // CubeMX GPIO init
        MX_TIM1_Init();  // CubeMX Timer init
        MX_ADC1_Init();  // CubeMX ADC1 init
        MX_ADC2_Init();  // CubeMX ADC2 init

        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

        HAL_ADCEx_InjectedStart_IT(&hadc1);
        HAL_ADCEx_InjectedStart_IT(&hadc2);

        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
        uint32_t sample = arr / 2; // Sample at 50% duty cycle
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample);

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Start PWM for ADC triggering

        // Initialize heartbeat
        //hb.start(tick);
    }

    // Run the main loop
    void loop() {
        while (true) {
            //hb.update(tick, led);
        }
    }
};

}
