#pragma once
#include "controlInputsFromUart.hpp"
#include "current_adc_hal.hpp"
#include "foc.hpp"
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
inline app::FOC foc{current_sense, telemetry, control_inputs, inverter};

inline void motor_control_isr()
{
    foc.run_isr();
}
} // namespace platform