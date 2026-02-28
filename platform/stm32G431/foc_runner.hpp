#pragma once
#include "bsp.hpp"
#include "controlInputsFromUart.hpp"
#include "current_adc_hal.hpp"
#include "foc.hpp"
#include "gate_driver_enable.hpp"
#include "inverter.hpp"
#include "telemetry.hpp"

extern "C"
{
#include "tim.h"
}

namespace platform
{

inline hal::CurrentSense current_sense;
inline ControlInputsFromUart control_inputs;
inline Telemetry telemetry;
inline hal::Inverter inverter(htim1);
inline hal::GateDriverEnable
    gate_enable({bsp::powerstage_enable_a().port, bsp::powerstage_enable_a().pin},
                {bsp::powerstage_enable_b().port, bsp::powerstage_enable_b().pin},
                {bsp::powerstage_enable_c().port, bsp::powerstage_enable_c().pin},
                {bsp::powerstage_enable_general().port, bsp::powerstage_enable_general().pin});

inline app::FOC foc{current_sense, telemetry, control_inputs, inverter, gate_enable};

inline void motor_control_isr()
{
    foc.run_isr();
}
} // namespace platform