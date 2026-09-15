#pragma once
#include "interfaces.hpp"
#include "powerstage_parameters.hpp"
#include <algorithm>
#include <cstdint>

extern "C"
{
#include "tim.h"
}

namespace hal
{
class Inverter : public app::IInverter
{
  public:
    explicit Inverter(TIM_HandleTypeDef& htim,
                      float pwmPeriod_s = 1.0f / 20000.0f,
                      float minMeasWindow_ns = bsp::powerstage_parameters.minSamplingWindow_ns,
                      float deadtime_ns = bsp::powerstage_parameters.deadtime_ns)
        : htim(&htim),
          minDuty((deadtime_ns * 1e-9f) / pwmPeriod_s),
          maxDuty(1.0f - ((minMeasWindow_ns * 1e-9f) / pwmPeriod_s))
    {
    }

    void
    set_phase_voltages(float va_V, float vb_V, float vc_V, float v_bus_V, bool isEnabled) override
    {
        float safe_vbus = (v_bus_V > 1.0f) ? v_bus_V : 12.0f;

        float duty_a = isEnabled ? std::clamp(0.5f + va_V / safe_vbus, minDuty, maxDuty) : 0.0f;
        float duty_b = isEnabled ? std::clamp(0.5f + vb_V / safe_vbus, minDuty, maxDuty) : 0.0f;
        float duty_c = isEnabled ? std::clamp(0.5f + vc_V / safe_vbus, minDuty, maxDuty) : 0.0f;

        const uint32_t arr = htim->Instance->ARR;

        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, static_cast<uint32_t>(duty_a * arr));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, static_cast<uint32_t>(duty_b * arr));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, static_cast<uint32_t>(duty_c * arr));
    }

    float get_min_duty() const { return minDuty; }
    float get_max_duty() const { return maxDuty; }

  private:
    TIM_HandleTypeDef* htim;
    float minDuty;
    float maxDuty;
};
} // namespace hal