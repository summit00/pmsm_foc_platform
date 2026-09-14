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
    explicit Inverter(TIM_HandleTypeDef& htim,
                      float pwmPeriod_s,
                      float minMeasWindow_ns = 2000.0f,
                      float minTurnOnTime_ns = 100.0f)
        : htim_(&htim),
          pwmPeriod_s_(pwmPeriod_s),
          minMeasWindow_ns_(minMeasWindow_ns),
          minTurnOnTime_ns_(minTurnOnTime_ns),
          minDuty_((minTurnOnTime_ns_ * 1e-9f) / pwmPeriod_s),
          maxDuty_(1.0f - ((minMeasWindow_ns_ * 1e-9f) / pwmPeriod_s))
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

        float duty_a = std::clamp(0.5f + va_V / safe_vbus, minDuty_, maxDuty_);
        float duty_b = std::clamp(0.5f + vb_V / safe_vbus, minDuty_, maxDuty_);
        float duty_c = std::clamp(0.5f + vc_V / safe_vbus, minDuty_, maxDuty_);

        const uint32_t arr = htim_->Instance->ARR;

        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, static_cast<uint32_t>(duty_a * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, static_cast<uint32_t>(duty_b * arr));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, static_cast<uint32_t>(duty_c * arr));
    }

    float get_min_duty() const { return minDuty_; }
    float get_max_duty() const { return maxDuty_; }
    float get_min_meas_window_ns() const { return minMeasWindow_ns_; }
    float get_min_turn_on_time_ns() const { return minTurnOnTime_ns_; }

  private:
    TIM_HandleTypeDef* htim_;
    float pwmPeriod_s_;
    float minMeasWindow_ns_;
    float minTurnOnTime_ns_;
    float minDuty_;
    float maxDuty_;
    bool is_moe_enabled_{false};
};

} // namespace hal
