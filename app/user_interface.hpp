#pragma once
#include <cstdint>

namespace app
{

struct UserInterface
{
    // Commands
    uint8_t mEnable = 0;
    uint8_t mMode = 0;
    float targetSpeed_rpm = 0.0f;
    float mAcceleration_rpm_s = 500.0f;
    float mIsAbs_mA = 0.0f;

    // Telemetry
    float actualSpeed_rpm = 0.0f;
    float busVoltage_V = 0.0f;
    float Id_A = 0.0f;
    float Iq_A = 0.0f;
    float IdRef_A = 0.0f;
    float IqRef_A = 0.0f;
    float ThetaEncoder_deg = 0.0f;
    float ThetaOpenLoop_deg = 0.0f;
    float actualSpeedEncoder_rpm = 0.0f;
};

} // namespace app
