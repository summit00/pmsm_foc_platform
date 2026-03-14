#pragma once
#include "VfControl.hpp"
#include "interfaces.hpp"
#include "runtime_measurement.hpp"
#include "theta_generator.hpp"
#include "transform.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <tuple>

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
                 const ICycleCounter& cycle_counter,
                 IEncoder& encoder)
        : current_sense(current_sense), telemetry(telemetry), control_inputs(control_inputs),
          inverter(inverter), gate_enable(gate_enable), encoder(encoder),
          runtime_measurement(cycle_counter)
    {
    }

    void run_isr()
    {
        PhaseCurrents currents_amps = current_sense.read_amps();
        ControlInputs inputs = control_inputs.read();
        target_speed_rpm = inputs.target_speed_rpm;
        // max_current_mA = inputs.max_current_mA;
        bool newEnabled = inputs.enable;

        if (newEnabled != isEnabled)
        {
            isEnabled = newEnabled;
            gate_enable.set_enable(isEnabled);

            if (!isEnabled)
            {
                inverter.set_phase_voltages(0.0f, 0.0f, 0.0f, vbus_V);
                theta_generator.reset();
            }
        }

        if (isEnabled)
        {
            runtime_measurement.start();
            theta_generator.update(mech_rpm_to_elec_rad_per_sec(target_speed_rpm, 4));
            mTheta = theta_generator.get_theta_rad();
            mOmega_rad_Hz = theta_generator.get_omega_rad_Hz();
            std::tie(mVa_V, mVb_V, mVc_V) = vf_control.update(mOmega_rad_Hz, mTheta);

            inverter.set_phase_voltages(mVa_V, mVb_V, mVc_V, vbus_V);
            runtime_measurement.stop();
            runtime1 = runtime_measurement.elapsed_cycles();
        }

        mTheta_encoder = encoder.read_raw();

        telemetry.publish5_i16(static_cast<int16_t>(currents_amps.ia_A * 1000.0f),
                               static_cast<int16_t>(currents_amps.ic_A * 1000.0f),
                               static_cast<int16_t>(mVa_V * 1000.0f),
                               static_cast<int16_t>(mTheta_encoder),
                               static_cast<int16_t>(mTheta * 1000.0f));
    }

    bool is_enabled() const
    {
        return isEnabled;
    }

    float mech_rpm_to_elec_rad_per_sec(int16_t rpm, int16_t pole_pairs) const
    {
        constexpr float rpm_to_rad_Hz = std::numbers::pi_v<float> / 30.0f;

        return static_cast<float>(rpm) * static_cast<float>(pole_pairs) * rpm_to_rad_Hz;
    }

  private:
    static constexpr float vbus_V = 24.0f;
    static constexpr float dt_s = 1.0f / 20000.0f; // ISR rate
    static constexpr float ramp_rate_rpm_per_s = 500.0f;

    ICurrentSense& current_sense;
    ITelemetry& telemetry;
    IControlInputs& control_inputs;
    IInverter& inverter;
    IEnableOutput& gate_enable;
    IEncoder& encoder;
    RuntimeMeasurement runtime_measurement;
    ThetaGenerator theta_generator{dt_s, mech_rpm_to_elec_rad_per_sec(ramp_rate_rpm_per_s, 4)};
    float motor_V_per_Hz{14000 * 4 / 60};
    VfControl vf_control{motor_V_per_Hz, 0.2f};

    bool isEnabled = false;
    int16_t target_speed_rpm = 0;

    float mTheta{};
    uint16_t mTheta_encoder{};
    float mOmega_rad_Hz{};

    uint32_t runtime1{};

    float mVa_V{};
    float mVb_V{};
    float mVc_V{};
};
} // namespace app