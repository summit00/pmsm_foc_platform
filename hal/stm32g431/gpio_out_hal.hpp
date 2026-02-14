#pragma once
#include <cstdint>
#include "interfaces.hpp"
#include "stm32g4xx_hal.h"

namespace hal {

class GpioOutHal final : public app::IDigitalOut {
public:
    GpioOutHal(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

}

