#pragma once
#include "powerstage_parameters.hpp"
#include <cstdint>

namespace hal
{

struct BoardSensorsConfig
{
    float adc_vref_V = bsp::powerstage_parameters.adc_vref_V;
    float shunt_resistor_ohm = bsp::powerstage_parameters.shunt_resistor_ohm;
    float current_amp_gain = bsp::powerstage_parameters.current_amp_gain;
    float vbus_r_top_ohm = bsp::powerstage_parameters.vbus_r_top_ohm;
    float vbus_r_bottom_ohm = bsp::powerstage_parameters.vbus_r_bottom_ohm;
    float ntc_pull_down_r = bsp::powerstage_parameters.ntc_pull_down_r;
    float ntc_r25 = bsp::powerstage_parameters.ntc_r25;
    float ntc_beta = bsp::powerstage_parameters.ntc_beta;

    uint16_t adc_ia_offset = bsp::powerstage_parameters.adc_ia_offset;
    uint16_t adc_ic_offset = bsp::powerstage_parameters.adc_ic_offset;

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
