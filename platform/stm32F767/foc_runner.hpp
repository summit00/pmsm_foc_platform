#pragma once
#include "adc.hpp"
#include "bsp.hpp"
#include "control.hpp"
#include "dwt_cycle_counter.hpp"
#include "encoder.hpp"
#include "gate_driver_enable.hpp"
#include "inverter.hpp"
#include "motor_params.hpp"
#include "usb_comm.hpp"

extern "C"
{
#include "tim.h"
}

namespace platform
{

inline app::MotorParams motor_params{.Rs_ohm = 0.1f,
                                     .RTotal_ohm = 0.1f,
                                     .Ld_H = 0.00016f,
                                     .Lq_H = 0.00016f,
                                     .flux_pm_Wb = 0.00408f,
                                     .polePairs = 4.0f,
                                     .encoderOffset_ticks = 295};

inline hal::ADCSense adc_sense;
inline hal::Inverter inverter(htim1);
inline hal::EncoderQEI encoder(htim4, 2000, 4);
inline app::UserInterface ui;

inline hal::GateDriverEnable
    gate_enable({bsp::powerstage_enable_a().port, bsp::powerstage_enable_a().pin},
                {bsp::powerstage_enable_b().port, bsp::powerstage_enable_b().pin},
                {bsp::powerstage_enable_c().port, bsp::powerstage_enable_c().pin},
                {bsp::powerstage_enable_general().port, bsp::powerstage_enable_general().pin});

constexpr float pwmPeriod_s = 1.0f / 20000.0f;

inline app::Control control{
    adc_sense, inverter, gate_enable, encoder, motor_params, ui, pwmPeriod_s};

inline void motor_control_isr()
{
    control.run_isr();

    TelemetrySample sample;
    sample.Udc_V = static_cast<int16_t>(ui.Udc_V * 100.0f);
    sample.demandSpeed_rpm = static_cast<int16_t>(ui.demandSpeed_rpm);
    sample.feedbackSpeed_rpm = static_cast<int16_t>(ui.feedbackSpeed_rpm);
    sample.encoderSpeed_rpm = static_cast<int16_t>(ui.encoderSpeed_rpm);
    sample.observerSpeed_rpm = static_cast<int16_t>(ui.observerSpeed_rpm);
    sample.Id_A = static_cast<int16_t>(ui.Id_A * 1000.0f);
    sample.Iq_A = static_cast<int16_t>(ui.Iq_A * 1000.0f);
    sample.encoderAngle_deg = static_cast<int16_t>(ui.encoderAngle_deg * 100.0f);
    sample.observerAngle_deg = static_cast<int16_t>(ui.observerAngle_deg * 100.0f);
    sample.angleError_deg = static_cast<int16_t>(ui.angleError_deg * 100.0f);

    g_usb_comm.push_sample(sample);
}

inline void calibrate_current_sense()
{
    adc_sense.calibrate_offset();
}

inline void init_encoder()
{
    encoder.start();
}

} // namespace platform
