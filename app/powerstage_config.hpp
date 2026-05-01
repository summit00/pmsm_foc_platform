#pragma once

namespace app
{

// Powerstage configuration for resistance compensation
// R_total = R_shunt * 3 (three-phase) + R_ds_on (MOSFETs)

struct PowerStageConfig
{
    float Rshunt_ohm = 0.0f;       // Shunt resistor value per phase
    float RdsOn_ohm = 0.0f;        // MOSFET on-resistance (total for path)
    float RtotalOffset_ohm = 0.0f; // Combined offset (Rshunt*3 + RdsOn)
};

// Get platform-specific powerstage configuration
// For STM32 hardware: includes shunt resistors and MOSFET Rds_on
// For host/simulation: no additional resistance (ideal)
inline PowerStageConfig getPowerStageConfig()
{
#if defined(TARGET_STM32)
    // Hardware values for typical 3-phase inverter:
    // - 0.33 ohm shunt per phase (3x = 1.0 ohm total)
    // - 0.73 ohm Rds_on for low-side MOSFETs
    return {0.33f, 0.73f, 1.03f};
#else
    // Simulation/host: no hardware resistance
    return {0.0f, 0.0f, 0.0f};
#endif
}

} // namespace app