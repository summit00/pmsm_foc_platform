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
    // Hardware values for 3-phase inverter:
    // - 0.01 ohm (10 mOhm) shunt per phase
    // - ~5 mOhm MOSFET Rds_on + trace resistance
    return {0.01f, 0.005f, 0.015f};
#else
    // Simulation/host: no hardware resistance
    return {0.0f, 0.0f, 0.0f};
#endif
}

} // namespace app