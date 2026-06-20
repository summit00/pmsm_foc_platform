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
    float mSineAmplitude = 0.0f;

    // Telemetry
    float Udc_V = 0.0f;
    float Id_A = 0.0f;
    float Iq_A = 0.0f;
    float demandSpeed_rpm = 0.0f;
    float openLoopSpeed_rpm = 0.0f;
    float encoderSpeed_rpm = 0.0f;
    float feedbackSpeed_rpm = 0.0f;
    float observerSpeed_rpm = 0.0f;
    float IdRef_A = 0.0f;
    float IqRef_A = 0.0f;
    float encoderAngle_deg = 0.0f;
    float observerAngle_deg = 0.0f;
    float angleError_deg = 0.0f;
};

} // namespace app
