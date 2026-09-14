#pragma once
#include <cstdint>
#include "interfaces.hpp"
#include "stm32h5xx_hal.h"

namespace hal {

class GpioOutHal final : public app::IDigitalOut {
public:
    GpioOutHal(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    void write(bool state) {
        HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void set() {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    }

    void reset() {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

}
