#pragma once
#include <cmath>
#include <cstdint>

namespace hal
{

struct BoardSensorsConfig
{
    float shunt_ohm = 0.33f;
    float opamp_gain = 1.53f;
    float adc_vref_V = 3.3f;
    uint16_t adc_counts_fs = 4095u;
    uint16_t adc_ia_offset = 2048u;
    uint16_t adc_ic_offset = 2048u;

    float vbus_r_high = 169000.0f; // R17
    float vbus_r_low = 9310.0f;    // R18

    float ntc_r25 = 10000.0f;        // 10K NTC
    float ntc_beta = 3435.0f;        // Standard for RS 742-8420
    float ntc_pull_down_r = 4700.0f; // R20

    float vbus_scale() const
    {
        return (vbus_r_high + vbus_r_low) / vbus_r_low;
    }

    float counts_to_amps() const
    {
        const float v_per_count = adc_vref_V / static_cast<float>(adc_counts_fs);
        return v_per_count / (shunt_ohm * opamp_gain);
    }
};

} // namespace hal