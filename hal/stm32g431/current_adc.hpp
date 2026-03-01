#pragma once
#include "current_sense_config.hpp"
#include "interfaces.hpp"
#include <array>
#include <cstdint>
#include <numeric>

namespace hal
{

class CurrentSense : public app::ICurrentSense
{
  public:
    // Called from ISR — just latches raw ADC values
    static void isr_update_currents(uint16_t ia, uint16_t ib)
    {
        ia_counts = ia;
        ib_counts = ib;
    }

    // Raw counts — app layer can use for debug/telemetry
    app::PhaseCurrentsRaw read_raw() const override
    {
        return {ia_counts, ib_counts};
    }

    // Calibrated amps — app layer uses this for FOC
    app::PhaseCurrents read_amps() const override
    {
        const float scale = cfg.counts_to_amps();
        return {static_cast<float>(static_cast<int32_t>(ia_counts) - cfg.adc_offset) * scale,
                static_cast<float>(static_cast<int32_t>(ib_counts) - cfg.adc_offset) * scale};
    }

    // Call once at startup before enabling the gate driver (no current flowing)
    // Averages N samples to find the true electrical zero
    void calibrate_offset() override
    {
        constexpr uint16_t N = 512;
        uint32_t sum_a = 0, sum_b = 0;

        for (uint16_t i = 0; i < N; ++i)
        {
            sum_a += ia_counts;
            sum_b += ib_counts;
            // Small busy-wait — calibration happens once in main(), not ISR
            for (volatile uint32_t d = 0; d < 1000; d++)
            {
            }
        }

        cfg.adc_offset = static_cast<uint16_t>((sum_a + sum_b) / (2u * N));
    }

  private:
    CurrentSenseConfig cfg{};

    static inline uint16_t ia_counts = 0;
    static inline uint16_t ib_counts = 0;
};

} // namespace hal