#pragma once
#include "interfaces.hpp"
#include "stm32h5xx_hal.h"

namespace hal
{

class DwtCycleCounter final : public app::ICycleCounter
{
  public:
    // Call once during system init — before using any ExecutionTimer
    static void enable()
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        DWT->CYCCNT = 0;
    }

    uint32_t now_cycles() const override
    {
        return DWT->CYCCNT;
    }

    uint32_t cycles_per_second() const override
    {
        return SystemCoreClock;
    }

    static void delay_us(uint32_t us)
    {
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = us * (SystemCoreClock / 1000000);
        while ((DWT->CYCCNT - start) < ticks)
        {
            __asm volatile("nop");
        }
    }
};

} // namespace hal
