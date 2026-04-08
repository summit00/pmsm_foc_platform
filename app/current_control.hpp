#pragma once
#include "pi_controller.hpp"
#include "precontrol.hpp"
#include <algorithm>
#include <tuple>

namespace app
{

class CurrentControl
{
  public:
    explicit CurrentControl(const float R_ohm,
                            const float Ld_H,
                            const float Lq_H,
                            const float flux_pm_Wb)
        : mRs(R_ohm), mLd(Ld_H), mLq(Lq_H), mPsi(flux_pm_Wb), pi_d(0.1f, 100.0f),
          pi_q(0.1f, 100.0f), precontrol(R_ohm, Ld_H, Lq_H, flux_pm_Wb)
    {
        set_params();
    }

    std::tuple<float, float> compute(float id_ref,
                                     float iq_ref,
                                     float id_meas,
                                     float iq_meas,
                                     float omega_rad_Hz,
                                     float outMin,
                                     float outMax)
    {

        auto [vd_ff, vq_ff] = precontrol.compute(id_ref, iq_ref, omega_rad_Hz);

        auto vd_pi_V = pi_d.compute(id_ref, id_meas, outMin, outMax);
        auto vq_pi_V = pi_q.compute(iq_ref, iq_meas, outMin, outMax);

        float vd = std::clamp(vd_pi_V + vd_ff, outMin, outMax);
        float vq = std::clamp(vq_pi_V + vq_ff, outMin, outMax);

        return {vd, vq};
    }

    void reset()
    {
        pi_d.reset();
        pi_q.reset();
    }

    void set_params()
    {
        auto [kp, ki] = pi_d.calculatePIGains(mRs, mLd, 1.0f / 20000.0f);
        pi_d.setGains(kp, ki);
        pi_q.setGains(kp, ki);
    }

  private:
    float mRs{};
    float mLd{};
    float mLq{};
    float mPsi{};
    PIController pi_d;
    PIController pi_q;
    Precontrol precontrol{mRs, mLd, mLq, mPsi};
};

} // namespace app