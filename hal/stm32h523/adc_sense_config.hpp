#pragma once
#include <cstdint>

namespace hal
{

struct BoardSensorsConfig
{
    float adc_vref_V = 3.3f;
    float shunt_resistor_ohm = 0.01f;
    float current_amp_gain = 20.0f;
    float vbus_r_top_ohm = 100000.0f;
    float vbus_r_bottom_ohm = 3300.0f;
    float ntc_pull_down_r = 4700.0f;
    float ntc_r25 = 10000.0f;
    float ntc_beta = 3455.0f;

    uint16_t adc_ia_offset = 2048;
    uint16_t adc_ic_offset = 2048;

    constexpr float counts_to_amps() const
    {
        return (adc_vref_V / 4095.0f) / (shunt_resistor_ohm * current_amp_gain);
    }

    constexpr float vbus_scale() const
    {
        return (vbus_r_top_ohm + vbus_r_bottom_ohm) / vbus_r_bottom_ohm;
    }
};

} // namespace hal
