#pragma once
#include "VfControl.hpp"
#include "current_control.hpp"
#include "interfaces.hpp"
#include "motor_params.hpp"
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
                 IInverter& inverter,
                 IEnableOutput& gate_enable,
                 const ICycleCounter& cycle_counter,
                 IEncoder& encoder,
                 const MotorParams& params)
        : current_sense(current_sense), inverter(inverter), gate_enable(gate_enable),
          encoder(encoder), runtime_measurement(cycle_counter), mParams(params),
          current_control(params)
    {
    }

    void run_isr()
    {
        PhaseCurrents currents_amps = current_sense.read_amps();
        mIdRef_A = max_current_mA / 1000.0f;

        if (newEnabled != isEnabled)
        {
            isEnabled = newEnabled;
            gate_enable.set_enable(static_cast<bool>(isEnabled));

            if (isEnabled == 0)
            {
                inverter.set_phase_voltages(0.0f, 0.0f, 0.0f, vbus_V);
                theta_generator.reset();
                resetFoc();
            }
        }

        if (isEnabled == 1)
        {
            runtime_measurement.start();

            float omega_target = mech_rpm_to_elec_rad_per_sec(target_speed_rpm, mParams.polePairs);
            theta_generator.update(omega_target);

            mTheta = theta_generator.get_theta_rad();
            mOmega_rad_Hz = theta_generator.get_omega_rad_Hz();

            std::tie(mIalpha_A, mIbeta_A) =
                transform.clarke(-currents_amps.ia_A, -currents_amps.ic_A);
            std::tie(mId_A, mIq_A) = transform.park(mIalpha_A, mIbeta_A, mTheta);
            std::tie(mUd_V, mUq_V) =
                current_control.compute(mIdRef_A,
                                        mIqRef_A,
                                        mId_A,
                                        mIq_A,
                                        mOmega_rad_Hz,
                                        -vbus_V * std::numbers::inv_sqrt3_v<float>,
                                        vbus_V * std::numbers::inv_sqrt3_v<float>);
            std::tie(mValpha_V, mVbeta_V) = transform.inversePark(mUd_V, mUq_V, mTheta);
            std::tie(mVa_V, mVb_V, mVc_V) = transform.inverseClarke(mValpha_V, mVbeta_V);
            inverter.set_phase_voltages(mVa_V, mVb_V, mVc_V, vbus_V);
            runtime_measurement.stop();
            runtime1 = runtime_measurement.elapsed_cycles();
        }

        mTheta_encoder = encoder.read_raw();
    }

    bool is_enabled() const
    {
        return isEnabled;
    }

    float mech_rpm_to_elec_rad_per_sec(int16_t rpm, float polePairs) const
    {
        constexpr float rpm_to_rad_Hz = std::numbers::pi_v<float> / 30.0f;
        return static_cast<float>(rpm) * polePairs * rpm_to_rad_Hz;
    }

  private:
    float elec_rad_per_sec_to_mech_rpm(float rad_per_sec_el, int16_t pole_pairs) const
    {
        constexpr float rads_to_rpm = 30.0f / std::numbers::pi_v<float>;

        return (rad_per_sec_el / static_cast<float>(pole_pairs)) * rads_to_rpm;
    }

    void resetFoc()
    {
        mId_A = 0.0f;
        mIq_A = 0.0f;
        mIalpha_A = 0.0f;
        mIbeta_A = 0.0f;
        mUd_V = 0.0f;
        mUq_V = 0.0f;
        mValpha_V = 0.0f;
        mVbeta_V = 0.0f;
    }

    static constexpr float vbus_V = 23.0f;
    static constexpr float dt_s = 1.0f / 20000.0f;
    static constexpr float ramp_rate_rpm_per_s = 500.0f;

    ICurrentSense& current_sense;
    IInverter& inverter;
    IEnableOutput& gate_enable;
    IEncoder& encoder;
    const MotorParams& mParams; // Shared motor data reference

    RuntimeMeasurement runtime_measurement;
    ThetaGenerator theta_generator{dt_s, mech_rpm_to_elec_rad_per_sec(ramp_rate_rpm_per_s, 4.0f)};
    Transforms transform;
    CurrentControl current_control;

    uint8_t isEnabled = 0;
    uint8_t newEnabled = 0;
    int16_t target_speed_rpm = 0;
    int32_t max_current_mA{};
    float mTheta{};
    uint16_t mTheta_encoder{};
    float mOmega_rad_Hz{};

    uint32_t runtime1{};

    float mVa_V{};
    float mVb_V{};
    float mVc_V{};

    float mIalpha_A{};
    float mIbeta_A{};
    float mId_A{};
    float mIq_A{};
    float mValpha_V{};
    float mVbeta_V{};
    float mUd_V{};
    float mUq_V{};
    float mIdRef_A{};
    float mIqRef_A{0.0f};
};
} // namespace app
