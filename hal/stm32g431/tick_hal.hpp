#pragma once
#include <cstdint>
#include "interfaces.hpp"
#include "stm32g4xx_hal.h"

namespace hal {

class TickHal final : public app::ITickMs {
public:
    TickHal() = default;

    uint32_t now_ms() const override {
        return HAL_GetTick();
    }
};

}
