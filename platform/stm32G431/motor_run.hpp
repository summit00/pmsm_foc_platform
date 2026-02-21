#pragma once
#include "controlInputsFromUart.hpp"
#include "current_adc_hal.hpp"
#include "motor_control.hpp"
#include "telemetry.hpp"

namespace platform
{
inline hal::CurrentSense current_sense;
inline Telemetry telemetry;
inline ControlInputsFromUart control_inputs;
inline app::MotorControl motor_control(current_sense, telemetry, control_inputs);

inline void motor_control_isr()
{
    motor_control.run_isr();
}
} // namespace platform