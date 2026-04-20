// emk_observer.hpp
#pragma once

#include "math.hpp"
#include "motor_params.hpp"
#include "sensor.hpp"
#include <algorithm>
#include <cmath>

namespace app
{

class EmkObserver : public ISensor
{
  public:
    EmkObserver(float dt_s, const MotorParams& params) : mDt_s(dt_s), mParams(params)
    {
    }

    void update() override
    {
        // Boundary Layer for Stability
        const float boundary = 0.05f; // 5mA precision

        // Current Estimation Error
        float s_alpha = mIalpha_est - mParams.Ialpha_A;
        float s_beta = mIbeta_est - mParams.Ibeta_A;

        // Super-Twisted SMO Control Law
        float ss_a = math::soft_sign(s_alpha, boundary);
        float ss_b = math::soft_sign(s_beta, boundary);

        float v_alpha = -mK1_smo * std::sqrt(std::abs(s_alpha) + 1e-6f) * ss_a + mZ_alpha;
        float v_beta = -mK1_smo * std::sqrt(std::abs(s_beta) + 1e-6f) * ss_b + mZ_beta;

        // Update the integral states Z
        mZ_alpha += -mK2_smo * ss_a * mDt_s;
        mZ_beta += -mK2_smo * ss_b * mDt_s;

        // EEMF Motor Model
        float saliency_alpha = mOmega_rad_Hz * (mParams.Ld_H - mParams.Lq_H) * mParams.Ibeta_A;
        float saliency_beta = -mOmega_rad_Hz * (mParams.Ld_H - mParams.Lq_H) * mParams.Ialpha_A;

        mIalpha_est +=
            (mParams.Ualpha_V - mParams.Rs_ohm * mIalpha_est + saliency_alpha + v_alpha) /
            mParams.Ld_H * mDt_s;
        mIbeta_est += (mParams.Ubeta_V - mParams.Rs_ohm * mIbeta_est + saliency_beta + v_beta) /
                      mParams.Ld_H * mDt_s;

        float theta_raw = std::atan2(mZ_alpha, -mZ_beta);

        float delta_theta = theta_raw - mTheta_rad;
        if (delta_theta > math::PI)
            delta_theta -= 2.0f * math::PI;
        if (delta_theta < -math::PI)
            delta_theta += 2.0f * math::PI;

        // Simple LPF for speed to handle the derivative noise
        const float alpha_speed = 0.05f;
        mOmega_rad_Hz = mOmega_rad_Hz * (1.0f - alpha_speed) + (delta_theta / mDt_s) * alpha_speed;

        mTheta_rad = theta_raw;
    }

    float getTheta_rad() const override
    {
        return mTheta_rad;
    }
    float getOmega_rad_Hz() const override
    {
        return mOmega_rad_Hz;
    }

    void reset() override
    {
        mTheta_rad = 0.0f;
        mOmega_rad_Hz = 0.0f;
        mOmega_int = 0.0f;
        mIalpha_est = 0.0f;
        mIbeta_est = 0.0f;
        mZ_alpha = 0.0f;
        mZ_beta = 0.0f;
    }

  private:
    float mDt_s;
    const MotorParams& mParams;

    float mK1_smo{1.0f};
    float mK2_smo{600.0f};

    // States
    float mIalpha_est{0.0f};
    float mIbeta_est{0.0f};
    float mZ_alpha{0.0f};
    float mZ_beta{0.0f};
    float mEalpha_filt{0.0f};
    float mEbeta_filt{0.0f};

    // Kinematic States
    float mTheta_rad{0.0f};
    float mOmega_rad_Hz{0.0f};
    float mOmega_int{0.0f};
};

} // namespace app