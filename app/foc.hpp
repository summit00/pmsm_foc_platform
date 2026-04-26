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
    explicit FOC(const MotorParams& params) : mParams(params), precontrol(params)
    {
        setCurrentControlGains();
    }

    std::tuple<float, float> runCurrentControl(float IdRef_A,
                                               float IqRef_A,
                                               float Id_A,
                                               float Iq_A,
                                               float omega_rad_Hz,
                                               float UsLimit_V,
                                               bool motor_enabled)
    {
        auto [Ud_ff, Uq_ff] = precontrol.compute(IdRef_A, IqRef_A, omega_rad_Hz);

        auto Ud_pi_V = pi_d.compute(IdRef_A, Id_A, -UsLimit_V, UsLimit_V);
        auto Uq_pi_V = pi_q.compute(IqRef_A, Iq_A, -UsLimit_V, UsLimit_V);

        float Ud = std::clamp(Ud_pi_V + Ud_ff, -UsLimit_V, UsLimit_V);
        float Uq = std::clamp(Uq_pi_V + Uq_ff, -UsLimit_V, UsLimit_V);

        std::tie(Ud, Uq) = DQLimiter::applyLimit(Ud, Uq, UsLimit_V);

        if (!motor_enabled)
        {
            Ud = 0.0f;
            Uq = 0.0f;
        }

        return {Ud, Uq};
    }

    const PIController& getIdController() const
    {
        return pi_d;
    }

    const PIController& getIqController() const
    {
        return pi_q;
    }

    std::tuple<float, float> runSpeedControl(
        float omegaRef_rad_Hz, float omega_rad_Hz, float outMin, float outMax, bool motor_enabled)
    {
        auto IqRef_A = PISpeed.compute(omegaRef_rad_Hz, omega_rad_Hz, outMin, outMax);

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
        auto [kp, ki] = pi_d.calculatePIGains(mParams.Rs_ohm, mParams.Ld_H, 1.0f / 20000.0f);
        pi_d.setGains(kp, ki);
        pi_q.setGains(kp, ki);
        PISpeed.setGains(0.05f, 0.0001f);
    }

  private:
    const MotorParams& mParams;
    Precontrol precontrol;
    PIController pi_d;
    PIController pi_q;
    PIController PISpeed;
};
} // namespace app
