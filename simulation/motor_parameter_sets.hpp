#pragma once

namespace sim
{

struct SimMotorParams
{
    // Electrical parameters
    float Rs_ohm;
    float Ld_H;
    float Lq_H;
    float fluxPm_Wb;
    float polePairs_count;

    // Mechanical parameters
    float inertia_kgm2;
    float viscousFriction_Nms;
};

namespace motors
{
inline constexpr SimMotorParams drone_motor{.Rs_ohm = 0.11f,
                                            .Ld_H = 0.000016f,
                                            .Lq_H = 0.000016f,
                                            .fluxPm_Wb = 0.00408f,
                                            .polePairs_count = 4.0f,
                                            .inertia_kgm2 = 0.00001f,
                                            .viscousFriction_Nms = 0.000001f};

inline constexpr SimMotorParams heavy_servo{.Rs_ohm = 0.1f,
                                            .Ld_H = 0.00016f,
                                            .Lq_H = 0.00016f,
                                            .fluxPm_Wb = 0.00408f,
                                            .polePairs_count = 4.0f,
                                            .inertia_kgm2 = 8.8e-6f,
                                            .viscousFriction_Nms = 0.0001f};
} // namespace motors

} // namespace sim