#pragma once
#include "adc_sense_config.hpp"
#include "interfaces.hpp"
#include <array>
#include <cstdint>
#include <numeric>

namespace hal
{

class ADCSense : public app::IADC
{
  public:
    static void isr_update(uint16_t ia, uint16_t ic, uint16_t vbus, uint16_t temp)
    {
        ia_counts = ia;
        ic_counts = ic;
        vbus_counts = vbus;
        temp_counts = temp;
    }

    app::PhaseCurrentsRaw read_raw() const override
    {
        return {ia_counts, ic_counts};
    }

    app::PhaseCurrents read_amps() const override
    {
        const float scale = cfg.counts_to_amps();
        return {-static_cast<float>(static_cast<int32_t>(ia_counts) - cfg.adc_ia_offset) * scale,
                -static_cast<float>(static_cast<int32_t>(ic_counts) - cfg.adc_ic_offset) * scale};
    }

    float read_bus_voltage() const override
    {
        float v_adc = (static_cast<float>(vbus_counts) / 4095.0f) * cfg.adc_vref_V;
        return v_adc * cfg.vbus_scale();
    }

    float read_temperature_celsius() const override
    {
        float v_out = (static_cast<float>(temp_counts) / 4095.0f) * cfg.adc_vref_V;
        if (v_out < 0.1f)
            return -273.15f;

        // R19 (NTC) and R20 (4.7k Pull-down)
        float r_ntc = cfg.ntc_pull_down_r * (cfg.adc_vref_V / v_out - 1.0f);

        // Beta equation calculation
        float steinhart = std::log(r_ntc / cfg.ntc_r25) / cfg.ntc_beta;
        steinhart += 1.0f / (25.0f + 273.15f);
        return (1.0f / steinhart) - 273.15f;
    }

    void calibrate_offset() override
    {
        constexpr uint16_t N = 1000;
        uint32_t sum_a = 0;
        uint32_t sum_b = 0;

        for (uint16_t i = 0; i < N; ++i)
        {
            sum_a += ia_counts;
            sum_b += ic_counts;
            // Small busy-wait — calibration happens once in main(), not ISR
            for (uint32_t d = 0; d < 1000; ++d)
            {
                // __asm volatile("nop");
            }
        }

        cfg.adc_ia_offset = static_cast<uint16_t>(sum_a / N);
        cfg.adc_ic_offset = static_cast<uint16_t>(sum_b / N);
    }

  private:
    BoardSensorsConfig cfg{};

    static inline uint16_t ia_counts = 0;
    static inline uint16_t ic_counts = 0;
    static inline uint16_t vbus_counts = 0;
    static inline uint16_t temp_counts = 0;
};

} // namespace hal