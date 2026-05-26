#pragma once
#include "interfaces.hpp"
#include "stm32f7xx_hal.h"

namespace hal
{

class DwtCycleCounter final : public app::ICycleCounter
{
  public:
    // Call once during system init — before using any ExecutionTimer
    static void enable()
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t now_cycles() const override
    {
        return DWT->CYCCNT;
    }

    uint32_t cycles_per_second() const override
    {
        return SystemCoreClock; // e.g. 216000000 on STM32F767
    }
};

} // namespace hal