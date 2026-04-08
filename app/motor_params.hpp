#pragma once

namespace app
{

struct MotorParams
{
    float Rs_ohm = 0.0f;
    float Ld_H = 0.0f;
    float Lq_H = 0.0f;
    float flux_pm_Wb = 0.0f;
    float polePairs = 0.0f;
};

} // namespace app
