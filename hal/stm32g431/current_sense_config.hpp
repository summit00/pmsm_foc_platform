#pragma once

namespace hal
{

// X-NUCLEO-IHM07M1 current sense chain
// Shunt: R43/R44/R45 = 0.33 Ohm, 1%
// Op-amp: TSV994IPT, overall AV = 1.53 (2.2k/2.2k diff-amp, schematic label confirmed)
// ADC: 12-bit, Vref = 3.3V
// Idle output biased to ~Vref/2 = 1.65V = 2048 counts

struct CurrentSenseConfig
{
    float shunt_ohm = 0.33f;
    float opamp_gain = 1.53f;
    float adc_vref_V = 3.3f;
    uint16_t adc_counts_fs = 4095u; // 12-bit full scale
    uint16_t adc_offset = 2048u;    // default mid-rail, refined by calibrate_offset()

    // Precomputed: counts-to-amps scale factor
    // V_per_count = Vref / counts_fs
    // I = (counts - offset) * V_per_count / (shunt * gain)
    float counts_to_amps() const
    {
        const float v_per_count = adc_vref_V / static_cast<float>(adc_counts_fs);
        return v_per_count / (shunt_ohm * opamp_gain);
        // = 3.3 / 4095 / (0.33 * 1.53) = ~0.001592 A/count  (~1.59 mA per LSB)
    }
};

} // namespace hal