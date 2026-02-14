#pragma once
#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "system_clock.hpp"
#include "heartbeat.hpp"
#include "tick_hal.hpp"
#include "gpio_out_hal.hpp"
#include "bsp.hpp"

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

        // Initialize heartbeat
        hb.start(tick);
    }

    // Run the main loop
    void loop() {
        while (true) {
            hb.update(tick, led);
        }
    }
};

}
