#pragma once
#include <cmath>
#include <cstdint>

namespace bsp
{

struct PowerstageParameters
{
    // Voltage limits (Volts)
    float max_voltage_V = 48.0f; // Overvoltage threshold
    float min_voltage_V = 10.0f; // Undervoltage threshold

    // Current limits (Amperes)
    float max_current_A = 2.5f; // Overcurrent threshold (X-NUCLEO-IHM07M1 max peak)

    // Temperature limits (Celsius)
    float max_temperature_C = 80.0f; // Overtemperature threshold

    // Inverter / Timing limits (nanoseconds)
    float deadtime_ns = 1000.0f;
    float minSamplingWindow_ns = 4000.0f;
    float minTurnOnTime_ns = 100.0f;

    // Derived PWM limits
    constexpr float min_duty(float pwmPeriod_s) const
    {
        return (deadtime_ns * 1e-9f) / pwmPeriod_s;
    }

    constexpr float max_duty(float pwmPeriod_s) const
    {
        return 1.0f - ((minSamplingWindow_ns * 1e-9f) / pwmPeriod_s);
    }

    // Analog sensing / Shunt / Opamp / Divider (IHM07M1)
    float adc_vref_V = 3.3f;
    uint16_t adc_counts_fs = 4095;
    float shunt_resistor_ohm = 0.33f;
    float current_amp_gain = 1.53f;
    uint16_t adc_ia_offset = 2048;
    uint16_t adc_ic_offset = 2048;

    float vbus_r_top_ohm = 169000.0f;
    float vbus_r_bottom_ohm = 9310.0f;

    float ntc_pull_down_r = 4700.0f;
    float ntc_r25 = 10000.0f;
    float ntc_beta = 3435.0f;

    constexpr float counts_to_amps() const
    {
        return (adc_vref_V / static_cast<float>(adc_counts_fs)) /
               (shunt_resistor_ohm * current_amp_gain);
    }

    constexpr float vbus_scale() const
    {
        return (vbus_r_top_ohm + vbus_r_bottom_ohm) / vbus_r_bottom_ohm;
    }
};

inline constexpr PowerstageParameters powerstage_parameters{};

} // namespace bsp
