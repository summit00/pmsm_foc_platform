#pragma once
// #include "VfController.hpp"
#include "interfaces.hpp"
#include "runtime_measurement.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace app
{
class FOC
{
  public:
    explicit FOC(ICurrentSense& current_sense,
                 ITelemetry& telemetry,
                 IControlInputs& control_inputs,
                 IInverter& inverter,
                 IEnableOutput& gate_enable,
                 const ICycleCounter& cycle_counter)
        : current_sense(current_sense), telemetry(telemetry), control_inputs(control_inputs),
          inverter(inverter), gate_enable(gate_enable), runtime_measurement(cycle_counter)
    {
    }

    void run_isr()
    {
        PhaseCurrentsRaw currents = current_sense.read_raw();
        PhaseCurrents currents_amps = current_sense.read_amps();
        ControlInputs inputs = control_inputs.read();

        bool newEnabled = inputs.enable;

        if (newEnabled != isEnabled)
        {
            isEnabled = newEnabled;
            gate_enable.set_enable(isEnabled);

            if (!isEnabled)
            {
                inverter.set_phase_voltages(0.0f, 0.0f, 0.0f, 24.0f);
            }
        }

        target_speed_rpm = inputs.target_speed_rpm;
        max_current_mA = inputs.max_current_mA;

        // Test DWT cycle counting
        runtime_measurement.start();
        float a = mTheta * 1.4142135f;      // multiply
        float b = a / 3.0f;                 // divide
        float c = std::sqrt(b * b + a * a); // sqrt + multiply + add
        m_sin_sink = c;
        runtime_measurement.stop();
        runtime2 = runtime_measurement.elapsed_cycles();

        telemetry.publish5_i16(static_cast<int16_t>(currents.ia_counts),
                               static_cast<int16_t>(currents.ib_counts),
                               static_cast<int16_t>(currents_amps.ia_A * 1000.0f),
                               static_cast<int16_t>(currents_amps.ib_A * 1000.0f),
                               isEnabled);
    }

    bool is_enabled() const
    {
        return isEnabled;
    }

  private:
    ICurrentSense& current_sense;
    ITelemetry& telemetry;
    IControlInputs& control_inputs;
    IInverter& inverter;
    IEnableOutput& gate_enable;
    RuntimeMeasurement runtime_measurement;

    bool isEnabled = false;
    int16_t target_speed_rpm = 0;
    int16_t max_current_mA = 0;

    float mTheta = 1.56f;
    float m_sin_sink = 0.0f;

    uint32_t runtime1{};
    uint32_t runtime2{};
};
} // namespace app