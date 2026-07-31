#pragma once
#include "adc.hpp"
#include "bsp.hpp"
#include "dwt_cycle_counter.hpp"
#include "foc_runner.hpp"
#include "gpio.h"
#include "gpio_out.hpp"
#include "heartbeat.hpp"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_adc_ex.h"
#include "tick.hpp"
#include "hal/usb_comm.hpp"

extern "C"
{
#include "adc.h"
#include "gpio.h"
#include "system_stm32f7xx.h"
#include "tim.h"
}

extern "C"
{
    void SystemClock_Config(void);
    void MX_GPIO_Init(void);
    void MX_TIM1_Init(void);
    void MX_TIM4_Init(void);
    void MX_ADC1_Init(void);
    void MX_ADC2_Init(void);
}

extern "C"
{
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim4;
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
        MX_GPIO_Init();
        MX_TIM1_Init();
        MX_TIM4_Init();
        MX_ADC1_Init();
        MX_ADC2_Init();

        // Clear the JAUTO bit on ADC2 to fix broken Dual ADC Injected Simultaneous mode.
        // CubeMX mistakenly enables Automatic Injected Conversion (JAUTO) on ADC2,
        // which prevents the simultaneous trigger from working.
        hadc2.Instance->CR1 &= ~ADC_CR1_JAUTO;

        // Set IRQ Priorities
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);

        // Start regular ADC conversions for both ADCs (required for F7 where calibration is
        // omitted)
        // HAL_ADC_Start(&hadc1);
        // HAL_ADC_Start(&hadc2);

        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
        uint32_t sample = arr - 10;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample);

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        HAL_ADCEx_InjectedStart(&hadc2);
        HAL_ADCEx_InjectedStart_IT(&hadc1);

        hal::DwtCycleCounter::enable();

        platform::init_encoder();

        platform::calibrate_current_sense();

        MX_USB_DEVICE_Init();
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
        platform::g_usb_comm.init();
        platform::g_usb_comm.setRxCallback([](const platform::RxCommand& c, void* ctx) {
            auto& ui = *static_cast<app::UserInterface*>(ctx);
            ui.mEnable = c.enable != 0;
            ui.mMode = static_cast<uint8_t>(c.mode);
            ui.targetSpeed_rpm = c.targetSpeed_rpm;
            ui.mAcceleration_rpm_s = c.accel_rpm_s;
            ui.mIsAbs_mA = c.isAbs_mA;
        }, &platform::ui);

        // Initialize heartbeat
        hb.start(tick);
    }

    // Run the main loop
    void loop()
    {
        hal::DwtCycleCounter cycle_counter;
        uint32_t last_usb = cycle_counter.now_cycles();
        while (true)
        {
            hb.update(tick, led);

            uint32_t now = cycle_counter.now_cycles();
            if ((now - last_usb) >= (cycle_counter.cycles_per_second() / 1000)) // ~1 ms
            {
                last_usb = now;
                // last_usb += cycle_counter.cycles_per_second() / 1000; // avoid drift
                platform::g_usb_comm.update();
            }
        }
    }
};
} // namespace app
