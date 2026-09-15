#pragma once
#include "adc.hpp"
#include "bsp.hpp"
#include "control.hpp"
#include "dwt_cycle_counter.hpp"
#include "encoder.hpp"
#include "gate_driver_enable.hpp"
#include "inverter.hpp"
#include "motor_params.hpp"
#include "runtime_measurement.hpp"
#include "comm/telemetry_manager.hpp"

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

constexpr float pwmPeriod_s = 1.0f / 20000.0f;

inline hal::ADCSense adc_sense;
inline hal::Inverter inverter(htim1, pwmPeriod_s, bsp::powerstage_parameters.minSamplingWindow_ns, bsp::powerstage_parameters.deadtime_ns);
inline hal::EncoderQEI encoder(htim4, 2000, 4);
inline app::UserInterface ui;

inline hal::GateDriverEnable
    gate_enable({bsp::powerstage_enable_a().port, bsp::powerstage_enable_a().pin},
                {bsp::powerstage_enable_b().port, bsp::powerstage_enable_b().pin},
                {bsp::powerstage_enable_c().port, bsp::powerstage_enable_c().pin},
                {bsp::powerstage_enable_general().port, bsp::powerstage_enable_general().pin});

inline app::Control control{
    adc_sense,
    inverter,
    gate_enable,
    encoder,
    motor_params,
    ui,
    pwmPeriod_s,
    {.overcurrent_threshold_A = bsp::powerstage_parameters.max_current_A,
     .overvoltage_threshold_V = bsp::powerstage_parameters.max_voltage_V,
     .undervoltage_threshold_V = bsp::powerstage_parameters.min_voltage_V,
     .overtemp_threshold_C = bsp::powerstage_parameters.max_temperature_C}};

inline hal::DwtCycleCounter cycle_counter;
inline app::RuntimeMeasurement foc_timer(cycle_counter);

inline void motor_control_isr()
{
    foc_timer.start();
    control.run_isr();
    foc_timer.stop();

    ui.runtimeTicks = static_cast<float>(foc_timer.elapsed_cycles());
    ui.runtime_us = foc_timer.elapsed_us();

    comm::g_telemetry_manager.capture_telemetry_isr();
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
