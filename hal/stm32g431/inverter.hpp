
#pragma once
#include "interfaces.hpp"
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
    explicit Inverter(TIM_HandleTypeDef& htim) : htim(&htim)
    {
    }

    void set_phase_voltages(float va_V, float vb_V, float vc_V, float v_bus_V) override
    {
        // For simplicity, let's assume the voltages are normalized to [0.0, 1.0]
        float duty_a = std::clamp(0.5f + va_V / v_bus_V, minDuty, maxDuty);
        float duty_b = std::clamp(0.5f + vb_V / v_bus_V, minDuty, maxDuty);
        float duty_c = std::clamp(0.5f + vc_V / v_bus_V, minDuty, maxDuty);

        const uint32_t arr = htim->Instance->ARR;

        htim->Instance->CCR1 = static_cast<uint32_t>(duty_a * static_cast<float>(arr));
        htim->Instance->CCR2 = static_cast<uint32_t>(duty_b * static_cast<float>(arr));
        htim->Instance->CCR3 = static_cast<uint32_t>(duty_c * static_cast<float>(arr));
    }

  private:
    TIM_HandleTypeDef* htim;
    float minDuty = 0.0f;
    float maxDuty = 0.9f;
};
} // namespace hal