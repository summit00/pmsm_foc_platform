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

inline constexpr SimMotorParams motor1{.Rs_ohm = 0.1f,
                                       .Ld_H = 0.00016f,
                                       .Lq_H = 0.00016f,
                                       .fluxPm_Wb = 0.00408f,
                                       .polePairs_count = 4.0f,
                                       .inertia_kgm2 = 8.8e-6f,
                                       .viscousFriction_Nms = 3.6e-6f};
} // namespace motors

} // namespace sim