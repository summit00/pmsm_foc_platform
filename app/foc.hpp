#pragma once
#include "dq_limiter.hpp"
#include "interfaces.hpp"
#include "motor_params.hpp"
#include "pi_controller.hpp"
#include "precontrol.hpp"
#include "runtime_measurement.hpp"
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
    explicit FOC(const MotorParams& params, float pwmPeriod_s)
        : mParams(params), precontrol(params), mPwmPeriod_s(pwmPeriod_s)
    {
        setCurrentControlGains();
    }

    const PIController& getIdController() const
    {
        return pi_d;
    }

    const PIController& getIqController() const
    {
        return pi_q;
    }

    std::tuple<float, float> runCurrentControl(float IdRef_A,
                                               float IqRef_A,
                                               float Id_A,
                                               float Iq_A,
                                               float omega_rad_Hz,
                                               float UsLimit_V,
                                               bool motor_enabled,
                                               bool isDrivingClosedLoop = true)
    {
        auto [Udff_V, Uqff_V] = precontrol.compute(IdRef_A, IqRef_A, omega_rad_Hz, UsLimit_V);

        float Ud = 0.0f;
        float Uq = 0.0f;

        if (isDrivingClosedLoop)
        {
            const float UdMax_V = 0.95f * UsLimit_V;
            Ud = pi_d.compute(IdRef_A, Id_A, Udff_V, -UdMax_V, UdMax_V);

            const float UqMax_V = std::sqrt(std::max(0.0f, UsLimit_V * UsLimit_V - Ud * Ud));
            Uq = pi_q.compute(IqRef_A, Iq_A, Uqff_V, -UqMax_V, UqMax_V);
        }
        else
        {
            Ud = pi_d.compute(IdRef_A, Id_A, Udff_V, -UsLimit_V, UsLimit_V);
            Uq = pi_q.compute(IqRef_A, Iq_A, Uqff_V, -UsLimit_V, UsLimit_V);

            const float Us_V = std::sqrt(Ud * Ud + Uq * Uq);
            if (Us_V > UsLimit_V)
            {
                Ud = Ud * UsLimit_V / Us_V;
                Uq = Uq * UsLimit_V / Us_V;
            }
        }

        if (!motor_enabled)
        {
            Ud = 0.0f;
            Uq = 0.0f;
        }

        return {Ud, Uq};
    }

    std::tuple<float, float>
    runSpeedControl(float omegaRef_rad_Hz, float omega_rad_Hz, float Iabs_A, bool motor_enabled)
    {
        auto IqRef_A = PISpeed.compute(omegaRef_rad_Hz, omega_rad_Hz, -Iabs_A, Iabs_A);

        if (!motor_enabled)
        {
            IqRef_A = 0.0f;
        }

        return {0.0f, IqRef_A};
    }

    void resetFoc()
    {
        pi_d.reset();
        pi_q.reset();
        PISpeed.reset();
    }

    void setCurrentControlGains()
    {
        auto [kp, ki] = pi_d.calculatePIGains(mParams.RTotal_ohm, mParams.Ld_H, mPwmPeriod_s);
        pi_d.setGains(kp, ki);
        pi_q.setGains(kp, ki);
        PISpeed.setGains(0.015f, 0.0003f);
    }

    void setCurrentControlGainsManual(float kp, float ki)
    {
        pi_d.setGains(kp, ki);
        pi_q.setGains(kp, ki);
    }

  private:
    const MotorParams& mParams;
    Precontrol precontrol;
    PIController pi_d;
    PIController pi_q;
    PIController PISpeed;
    float mPwmPeriod_s;
};
} // namespace app
