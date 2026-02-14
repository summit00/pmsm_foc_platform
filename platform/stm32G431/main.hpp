#pragma once
#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "heartbeat.hpp"
#include "tick_hal.hpp"
#include "gpio_out_hal.hpp"
#include "bsp.hpp"
extern "C" {
    void SystemClock_Config(void);
    void MX_GPIO_Init(void);
    // Add others as you enable them in CubeMX
}
extern "C" {
#include "system_stm32g4xx.h" // CubeMX clock
#include "gpio.h"             // CubeMX GPIO init
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
