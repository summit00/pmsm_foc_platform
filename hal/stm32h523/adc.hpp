#pragma once
#include "adc_sense_config.hpp"
#include "dwt_cycle_counter.hpp"
#include "interfaces.hpp"
#include <array>
#include <cmath>
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
        return {static_cast<float>(static_cast<int32_t>(ia_counts) - cfg.adc_ia_offset) * scale,
                static_cast<float>(static_cast<int32_t>(ic_counts) - cfg.adc_ic_offset) * scale};
    }

    float read_bus_voltage() const override
    {
        float v_adc = (static_cast<float>(vbus_counts) / 4095.0f) * cfg.adc_vref_V;
        return v_adc * cfg.vbus_scale();
    }

    float read_temperature_celsius() const override
    {
        if (temp_counts == 0)
        {
            return 25.0f; // Default room temperature when no sensor is mapped
        }

        float v_out = (static_cast<float>(temp_counts) / 4095.0f) * cfg.adc_vref_V;
        if (v_out < 0.1f)
            return 25.0f;

        float r_ntc = cfg.ntc_pull_down_r * (cfg.adc_vref_V / v_out - 1.0f);
        float steinhart = std::log(r_ntc / cfg.ntc_r25) / cfg.ntc_beta;
        steinhart += 1.0f / (25.0f + 273.15f);
        return (1.0f / steinhart) - 273.15f;
    }

    void calibrate_offset() override
    {
        constexpr uint32_t N = 4000;
        uint32_t sum_a = 0;
        uint32_t sum_c = 0;

        for (uint32_t i = 0; i < N; ++i)
        {
            sum_a += ia_counts;
            sum_c += ic_counts;
            hal::DwtCycleCounter::delay_us(50);
        }

        cfg.adc_ia_offset = static_cast<uint16_t>(sum_a / N);
        cfg.adc_ic_offset = static_cast<uint16_t>(sum_c / N);
    }

  private:
    BoardSensorsConfig cfg{};

    static inline volatile uint16_t ia_counts = 0;
    static inline volatile uint16_t ic_counts = 0;
    static inline volatile uint16_t vbus_counts = 0;
    static inline volatile uint16_t temp_counts = 0;
};

} // namespace hal
