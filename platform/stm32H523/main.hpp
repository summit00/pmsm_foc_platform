#pragma once

#include "bsp.hpp"
#include "dwt_cycle_counter.hpp"
#include "gpio_out.hpp"
#include "heartbeat.hpp"
#include "stm32h5xx_hal.h"
#include "tick.hpp"

#if __has_include("app_usbx_device.h")
#include "hal/stm32h523/usbx_cdc_transfer.hpp"
#include "comm/protocol_handler.hpp"
#include "foc_runner.hpp"
#define HAS_USB_CDC 1
#else
#define HAS_USB_CDC 0
#endif

extern "C"
{
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb.h"
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI3_Init(void);
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

#if HAS_USB_CDC
namespace platform
{
    inline hal::UsbxCdcTransfer g_usbx_cdc_transfer;
    inline comm::ProtocolHandler g_protocol_handler(g_usbx_cdc_transfer, comm::g_telemetry_manager);
}
#endif

namespace app
{

struct MainApp
{
    hal::TickHal tick;
    hal::GpioOutHal status_led;
    hal::GpioOutHal error_led;
    app::Heartbeat hb;

    MainApp()
        : status_led(bsp::status_led().port, bsp::status_led().pin),
          error_led(bsp::error_led().port, bsp::error_led().pin),
          hb(500)
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init();
        MX_SPI3_Init();
        MX_TIM1_Init();
        MX_TIM4_Init();
        MX_ADC1_Init();
        MX_ADC2_Init();

        // Reconfigure ADC1 Dual MultiMode to Injected Simultaneous
        // (CubeMX default is REGSIMULT_ALTERTRIG, which prevents ADC2 Injected Conversions from triggering)
        ADC_MultiModeTypeDef multimode{};
        multimode.Mode = ADC_DUALMODE_INJECSIMULT;
        multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
        multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
        if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
        {
            Error_Handler();
        }

        hal::DwtCycleCounter::enable();

        // Initialize DRV8353 gate driver via SPI3 FIRST
        // This wakes up the DRV8353 (ENABLE=HIGH) and powers up the CSAs with VREF/2 bias
        bool drv_ok = platform::init_gate_driver();
        platform::ui.drvInitOk = drv_ok ? 1.0f : 0.0f;
        if (!drv_ok)
        {
            platform::ui.errorState = 1.0f; // Indicate SPI init error
            error_led.set();
        }
        else
        {
            error_led.reset();
        }

        // Allow CSA analog outputs to stabilize after wake-up
        HAL_Delay(10);

        // Configure ADC Interrupt Priorities (ADC1 triggers ISR for dual injected mode)
        HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC1_IRQn);
        HAL_NVIC_DisableIRQ(ADC2_IRQn);

        // Hardware ADC self-calibration
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

        // Configure TIM1 Channel 4 compare trigger point near peak (ARR - 10)
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
        uint32_t sample = (arr > 10) ? (arr - 10) : (arr / 2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sample);

        // Start Dual Simultaneous Injected Conversions
        HAL_ADCEx_InjectedStart(&hadc2);
        HAL_ADCEx_InjectedStart_IT(&hadc1);

        // Start TIM1 PWM outputs (High-Side & Low-Side) and trigger channel
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
        __HAL_TIM_MOE_DISABLE(&htim1);

        // Start QEI Encoder Timer
        platform::init_encoder();

        // Calibrate Current Sense ADC offsets with 4000 live samples
        // (CSAs are now active and outputting 1.65V / 2048 counts)
        platform::calibrate_current_sense();

        // Initialize heartbeat status LED
        hb.start(tick);

#if HAS_USB_CDC
        platform::g_usbx_cdc_transfer.init(hpcd_USB_DRD_FS);

        comm::g_telemetry_manager.init();
        platform::g_protocol_handler.setRxCallback([](const comm::ProtocolHandler::RxCommand& c, void* ctx) {
            auto& ui = *static_cast<app::UserInterface*>(ctx);
            ui.mEnable = c.enable != 0;
            ui.mMode = static_cast<uint8_t>(c.mode);
            ui.targetSpeed_rpm = c.targetSpeed_rpm;
            ui.mAcceleration_rpm_s = c.accel_rpm_s;
            ui.mIsAbs_mA = c.isAbs_mA;
            ui.rxPackets += 1.0f;
        }, &platform::ui);
#endif
    }

    // Run the main loop
    void loop()
    {
#if HAS_USB_CDC
        hal::DwtCycleCounter cycle_counter;
        uint32_t last_usb = cycle_counter.now_cycles();
        uint32_t last_diag = cycle_counter.now_cycles();
        while (true)
        {
            hb.update(tick, status_led);

            // Continuously pump USB hardware state machines and process incoming host packets
            platform::g_usbx_cdc_transfer.poll_rx();

            uint32_t now = cycle_counter.now_cycles();

            // Transmit telemetry frames at 1 ms (1000 Hz) interval
            if ((now - last_usb) >= (cycle_counter.cycles_per_second() / 1000))
            {
                last_usb = now;
                platform::g_protocol_handler.update();
            }

            if ((now - last_diag) >= (cycle_counter.cycles_per_second() / 100)) // ~10 ms rate
            {
                last_diag = now;

                // Monitor DRV8353 Hardware Fault Pin (PB12 active LOW) with edge-triggered SPI read
                static bool prev_fault_active = false;
                bool fault_active = platform::gate_driver.is_fault_pin_active();
                if (fault_active && !prev_fault_active)
                {
                    auto faults = platform::gate_driver.read_faults();
                    platform::ui.drvFault1 = static_cast<float>(faults.raw_status_1);
                    platform::ui.drvFault2 = static_cast<float>(faults.raw_status_2);
                    platform::ui.errorState = 2.0f; // DRV8353 hardware fault asserted
                }
                else if (!fault_active && prev_fault_active)
                {
                    platform::ui.errorState = 0.0f; // Fault cleared
                    platform::ui.drvFault1 = 0.0f;
                    platform::ui.drvFault2 = 0.0f;
                }
                prev_fault_active = fault_active;

                // Update UI diagnostics
                platform::ui.cmdEnable = platform::ui.mEnable ? 1.0f : 0.0f;
                platform::ui.cmdMode = static_cast<float>(platform::ui.mMode);

                // Update Error LED (PD5): illuminate if any fault is active
                bool has_error = (platform::ui.errorState != 0.0f) || fault_active;
                error_led.write(has_error);
            }
        }
#else
        while (true)
        {
            hb.update(tick, status_led);
        }
#endif
    }
};

} // namespace app
