#pragma once
// #include "VfController.hpp"
#include "interfaces.hpp"
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
                 IEnableOutput& gate_enable)
        : current_sense(current_sense), telemetry(telemetry), control_inputs(control_inputs),
          inverter(inverter), gate_enable(gate_enable)
    {
    }

    void run_isr()
    {
        PhaseCurrentsRaw currents = current_sense.read_raw();
        ControlInputs inputs = control_inputs.read();

        bool newEnabled = inputs.enable;

        if (newEnabled != isEnabled)
        {
            isEnabled = newEnabled;
            gate_enable.set_enable(isEnabled);

            if (!isEnabled)
            {
                m_currentRpm = 0.0f;
                m_theta = 0.0f;
                inverter.set_phase_voltages(0.0f, 0.0f, 0.0f, 24.0f);
            }
        }

        target_speed_rpm = inputs.target_speed_rpm;
        max_current_mA = inputs.max_current_mA;

        if (isEnabled)
        {
            setTarget((float)target_speed_rpm, 5.0f);
            // 1. Ramp Generator
            float rpmStep = (m_targetRpm - m_currentRpm) * (m_dt / m_rampTime);

            if (std::abs(rpmStep) > std::abs(m_targetRpm - m_currentRpm))
            {
                m_currentRpm = m_targetRpm;
            }
            else
            {
                m_currentRpm += rpmStep;
            }

            // 2. RPM to Electrical Frequency (Hz)
            float elecFreqHz = (m_currentRpm * (float)m_polePairs) / 60.0f;

            // 3. V/F Scaling
            float slope = (m_ratedVoltage - m_voltageBoost) / m_ratedFreqHz;
            float vAmplitude = m_voltageBoost + (slope * std::abs(elecFreqHz));
            vAmplitude = std::min(vAmplitude, m_ratedVoltage);

            // 4. Angle Integration
            float omega = 2.0f * std::numbers::pi_v<float> * elecFreqHz;
            m_theta += omega * m_dt;

            // Wrap Theta [0, 2PI]
            m_theta = std::fmod(m_theta, 2.0f * std::numbers::pi_v<float>);
            if (m_theta < 0)
                m_theta += 2.0f * std::numbers::pi_v<float>;

            // 5. Phase Voltage Generation
            const float phaseShift = 2.0f * std::numbers::pi_v<float> / 3.0f;
            float Ua = vAmplitude * std::cos(m_theta);
            float Ub = vAmplitude * std::cos(m_theta - phaseShift);
            float Uc = vAmplitude * std::cos(m_theta - (2.0f * phaseShift));
            // auto [Ua, Ub, Uc] = open_loop_vf.update();
            inverter.set_phase_voltages(Ua, Ub, Uc, 24.0f);
        }
        else
        {
            m_currentRpm = 0.0f;
            m_theta = 0.0f;
            inverter.set_phase_voltages(0.0f, 0.0f, 0.0f, 24.0f);

            // Update open loop V/F controller
            setPhysics(3, 24.0f, 3000.0f, 2.0f);
            setSamplingTime(1.0f / 20000.0f);
        }

        telemetry.publish5_i16(currents.ia_counts,
                               currents.ib_counts,
                               isEnabled * 1000,
                               target_speed_rpm,
                               m_currentRpm);
    }

    void setTarget(float targetRpm, float rampTime)
    {
        m_targetRpm = targetRpm;
        m_rampTime = std::max(rampTime, 0.0001f);
    }

    void setPhysics(int polePairs, float ratedVoltage, float ratedSpeedRpm, float voltageBoost)
    {
        m_polePairs = polePairs;
        m_ratedVoltage = ratedVoltage;
        m_voltageBoost = voltageBoost;

        // Calculate internal rated frequency: f = (RPM * PP) / 60
        m_ratedFreqHz = (ratedSpeedRpm * (float)m_polePairs) / 60.0f;

        // Prevent division by zero in the update loop
        if (m_ratedFreqHz <= 0.0f)
            m_ratedFreqHz = 1.0f;
    }

    void setSamplingTime(float dt)
    {
        m_dt = dt;
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
    // OpenLoopVf& open_loop_vf;

    bool isEnabled = false;
    int16_t target_speed_rpm = 0;
    int16_t max_current_mA = 0;

    int m_polePairs = 1;
    float m_ratedVoltage = 0.0f;
    float m_ratedFreqHz = 1.0f;
    float m_voltageBoost = 0.0f;
    float m_dt = 0.001f;

    float m_targetRpm = 0.0f;
    float m_rampTime = 1.0f;

    float m_currentRpm = 0.0f;
    float m_theta = 0.0f;
};
} // namespace app