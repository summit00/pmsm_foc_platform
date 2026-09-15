#pragma once
#include "powerstage_parameters.hpp"
#include <cmath>
#include <cstdint>

namespace hal
{

struct BoardSensorsConfig
{
    float shunt_ohm = bsp::powerstage_parameters.shunt_resistor_ohm;
    float opamp_gain = bsp::powerstage_parameters.current_amp_gain;
    float adc_vref_V = bsp::powerstage_parameters.adc_vref_V;
    uint16_t adc_counts_fs = bsp::powerstage_parameters.adc_counts_fs;
    uint16_t adc_ia_offset = bsp::powerstage_parameters.adc_ia_offset;
    uint16_t adc_ic_offset = bsp::powerstage_parameters.adc_ic_offset;

    float vbus_r_high = bsp::powerstage_parameters.vbus_r_top_ohm;
    float vbus_r_low = bsp::powerstage_parameters.vbus_r_bottom_ohm;

    float ntc_r25 = bsp::powerstage_parameters.ntc_r25;
    float ntc_beta = bsp::powerstage_parameters.ntc_beta;
    float ntc_pull_down_r = bsp::powerstage_parameters.ntc_pull_down_r;

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