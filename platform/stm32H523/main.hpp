#pragma once

#include "bsp.hpp"
#include "dwt_cycle_counter.hpp"
#include "gpio_out.hpp"
#include "heartbeat.hpp"
#include "stm32h5xx_hal.h"
#include "tick.hpp"

extern "C"
{
#include "gpio.h"
#include "main.h"
void SystemClock_Config(void);
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

        hal::DwtCycleCounter::enable();

        // Initialize heartbeat
        hb.start(tick);
    }

    // Run the main loop
    void loop()
    {
        while (true)
        {
            hb.update(tick, led);
        }
    }
};

} // namespace app
