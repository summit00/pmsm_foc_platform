#pragma once
#include "adc.hpp"
#include "bsp.hpp"
#include "control.hpp"
#include "dwt_cycle_counter.hpp"
#include "encoder.hpp"
#include "gate_driver_enable.hpp"
#include "inverter.hpp"
#include "motor_params.hpp"

extern "C"
{
#include "tim.h"
}

namespace platform
{

inline app::MotorParams motor_params{.Rs_ohm = 0.1f,
                                     .Ld_H = 0.00016f,
                                     .Lq_H = 0.00016f,
                                     .flux_pm_Wb = 0.00408f,
                                     .polePairs = 4.0f,
                                     .encoderOffset_ticks = 295};

inline hal::ADCSense adc_sense;
inline hal::Inverter inverter(htim1);
inline hal::EncoderQEI encoder(htim2, 2000, 4);
inline app::UserInterface ui;

inline hal::GateDriverEnable
    gate_enable({bsp::powerstage_enable_a().port, bsp::powerstage_enable_a().pin},
                {bsp::powerstage_enable_b().port, bsp::powerstage_enable_b().pin},
                {bsp::powerstage_enable_c().port, bsp::powerstage_enable_c().pin},
                {bsp::powerstage_enable_general().port, bsp::powerstage_enable_general().pin});

inline app::Control control{adc_sense, inverter, gate_enable, encoder, motor_params, ui};

inline void motor_control_isr()
{
    control.run_isr();
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
