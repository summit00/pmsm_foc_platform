#pragma once
#include "interfaces.hpp"
#include <algorithm>
#include <cstdint>
#include "stm32h5xx_hal.h"

namespace hal
{

class Inverter : public app::IInverter
{
  public:
    explicit Inverter(TIM_HandleTypeDef& htim) : htim_(&htim)
    {
    }

    void
    set_phase_voltages(float va_V, float vb_V, float vc_V, float v_bus_V, bool isEnabled) override
    {
        float duty_a = isEnabled ? std::clamp(0.5f + va_V / v_bus_V, minDuty, maxDuty) : 0.0f;
        float duty_b = isEnabled ? std::clamp(0.5f + vb_V / v_bus_V, minDuty, maxDuty) : 0.0f;
        float duty_c = isEnabled ? std::clamp(0.5f + vc_V / v_bus_V, minDuty, maxDuty) : 0.0f;

        const uint32_t arr = htim_->Instance->ARR;

        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, static_cast<uint32_t>(duty_a * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, static_cast<uint32_t>(duty_b * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, static_cast<uint32_t>(duty_c * arr));
    }

  private:
    TIM_HandleTypeDef* htim_;
    float minDuty = 0.0f;
    float maxDuty = 0.95f;
};

} // namespace hal
