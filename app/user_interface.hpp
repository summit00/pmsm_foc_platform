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
    float Udc_V = 0.0f;
    float demandSpeed_rpm = 0.0f;
    float feedbackSpeed_rpm = 0.0f;
    float encoderSpeed_rpm = 0.0f;
    float observerSpeed_rpm = 0.0f;
    float openLoopSpeed_rpm = 0.0f;
    float Id_A = 0.0f;
    float Iq_A = 0.0f;
    float IdRef_A = 0.0f;
    float IqRef_A = 0.0f;
    float Ud_V = 0.0f;
    float Uq_V = 0.0f;
    float encoderAngle_deg = 0.0f;
    float observerAngle_deg = 0.0f;
    float angleError_deg = 0.0f;
    float Ialpha_A = 0.0f;
    float Ibeta_A = 0.0f;
    float Ualpha_V = 0.0f;
    float Ubeta_V = 0.0f;
    float temp_C = 0.0f;
    float errorState = 0.0f;
    float autoSetupState = 0.0f;

    // DRV8353 & Diagnostics
    float drvFault1 = 0.0f;
    float drvFault2 = 0.0f;
    float drvInitOk = 0.0f;
    float cmdEnable = 0.0f;
    float cmdMode = 0.0f;
    float rxPackets = 0.0f;
    float driveState = 0.0f;

    // Motor Parameters (Identified / Configured)
    float Rs_ohm = 0.0f;
    float Ld_uH = 0.0f;
    float Lq_uH = 0.0f;
    float flux_pm_mWb = 0.0f;
    float encoderOffset_ticks = 0.0f;

    // Encoder Diagnostics
    float encoderIndexFound = 0.0f;
    float encoderRawTicks = 0.0f;

    // Runtime Measurement
    float runtimeTicks = 0.0f;
    float runtime_us = 0.0f;
};

} // namespace app
