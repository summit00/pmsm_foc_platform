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
        if (!isEnabled)
        {
            if (is_moe_enabled_)
            {
                __HAL_TIM_MOE_DISABLE(htim_);
                is_moe_enabled_ = false;
            }
            __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, 0);
            return;
        }

        if (!is_moe_enabled_)
        {
            __HAL_TIM_MOE_ENABLE(htim_);
            is_moe_enabled_ = true;
        }

        // Safe bus voltage to prevent NaN/Infinity division
        float safe_vbus = (v_bus_V > 1.0f) ? v_bus_V : 12.0f;

        float duty_a = std::clamp(0.5f + va_V / safe_vbus, minDuty, maxDuty);
        float duty_b = std::clamp(0.5f + vb_V / safe_vbus, minDuty, maxDuty);
        float duty_c = std::clamp(0.5f + vc_V / safe_vbus, minDuty, maxDuty);

        const uint32_t arr = htim_->Instance->ARR;

        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, static_cast<uint32_t>(duty_a * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, static_cast<uint32_t>(duty_b * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, static_cast<uint32_t>(duty_c * arr));
    }

  private:
    TIM_HandleTypeDef* htim_;
    bool is_moe_enabled_ = false;
    float minDuty = 0.05f;
    float maxDuty = 0.95f;
};

} // namespace hal
