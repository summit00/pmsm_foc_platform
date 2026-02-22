
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
class Inverter(TIM_HandleTypeDef& htim)
{
  public:
    explicit Inverter(TIM_HandleTypeDef& htim) : htim(htim)
    {
    }

    void set_phase_voltages(const app::PhaseVoltages& voltages, float v_bus_V)
    {
        // For simplicity, let's assume the voltages are normalized to [0.0, 1.0]
        float duty_a = std::clamp(0.5f + voltages.va_V / v_bus_V, minDuty, maxDuty);
        float duty_b = std::clamp(0.5f + voltages.vb_V / v_bus_V, minDuty, maxDuty);
        float duty_c = std::clamp(0.5f + voltages.vc_V / v_bus_V, minDuty, maxDuty);

        const uint32_t arr = htim->Instance->Arr;

        htim->Instance->CCR1 = static_cast<uint32_t>(duty_a * arr);
        htim->Instance->CCR2 = static_cast<uint32_t>(duty_b * arr);
        htim->Instance->CCR3 = static_cast<uint32_t>(duty_c * arr);
    }

  private:
    TIM_HandleTypeDef* htim;
    float minDuty = 0.0f;
    float maxDuty = 0.9f;
};
} // namespace hal