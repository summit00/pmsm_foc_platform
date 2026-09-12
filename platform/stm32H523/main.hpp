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
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "usb.h"
void SystemClock_Config(void);
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
    hal::GpioOutHal led;
    app::Heartbeat hb;

    MainApp() : led(bsp::status_led().port, bsp::status_led().pin), hb(500)
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init();
        MX_SPI3_Init();

        hal::DwtCycleCounter::enable();

        // Initialize DRV8353 gate driver via SPI3
        bool drv_ok = platform::init_gate_driver();
        if (!drv_ok)
        {
            platform::ui.errorState = 1.0f; // Indicate SPI init error in telemetry
        }

        // Initialize heartbeat first so status LED blinks immediately
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
        }, &platform::ui);
#endif
    }

    // Run the main loop
    void loop()
    {
#if HAS_USB_CDC
        hal::DwtCycleCounter cycle_counter;
        uint32_t last_usb = cycle_counter.now_cycles();
        while (true)
        {
            hb.update(tick, led);
            platform::g_usbx_cdc_transfer.poll_tasks();

            uint32_t now = cycle_counter.now_cycles();
            if ((now - last_usb) >= (cycle_counter.cycles_per_second() / 1000)) // ~1 ms
            {
                last_usb = now;
                platform::g_usbx_cdc_transfer.poll_rx();

                // Telemetry feedback for testing USB transfer & commands
                platform::ui.Udc_V = 24.0f;
                if (platform::ui.mEnable)
                {
                    platform::ui.demandSpeed_rpm = platform::ui.targetSpeed_rpm;
                    platform::ui.feedbackSpeed_rpm = platform::ui.targetSpeed_rpm;
                    platform::ui.encoderSpeed_rpm = platform::ui.targetSpeed_rpm;
                    platform::ui.observerSpeed_rpm = platform::ui.targetSpeed_rpm;
                    platform::ui.Iq_A = platform::ui.mIsAbs_mA / 1000.0f;
                }
                else
                {
                    platform::ui.demandSpeed_rpm = 0.0f;
                    platform::ui.feedbackSpeed_rpm = 0.0f;
                    platform::ui.encoderSpeed_rpm = 0.0f;
                    platform::ui.observerSpeed_rpm = 0.0f;
                    platform::ui.Iq_A = 0.0f;
                }
                // Monitor DRV8353 Hardware Fault Pin (PB12 active LOW)
                if (platform::gate_driver.is_fault_pin_active())
                {
                    platform::ui.errorState = 2.0f; // DRV8353 hardware fault asserted
                }
                else if (platform::ui.errorState == 2.0f)
                {
                    platform::ui.errorState = 0.0f; // Fault cleared
                }

                // Push samples to telemetry buffer and send batch to host
                comm::g_telemetry_manager.capture_telemetry_isr();
                platform::g_protocol_handler.update();
            }
        }
#else
        while (true)
        {
            hb.update(tick, led);
        }
#endif
    }
};

} // namespace app
