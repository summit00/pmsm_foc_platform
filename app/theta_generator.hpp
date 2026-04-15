// theta_generator.hpp
#pragma once
#include "math.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace app
{

class ThetaGenerator
{
  public:
    ThetaGenerator(float dt_s, const float& rampRateRef_rad_Hz2)
        : mDt_s(dt_s), mRampRateRef_rad_Hz2(rampRateRef_rad_Hz2)
    {
    }

    void update(float targetOmega_rad_Hz)
    {
        if (mOmega_rad_Hz < targetOmega_rad_Hz)
        {
            mOmega_rad_Hz =
                std::min(mOmega_rad_Hz + mRampRateRef_rad_Hz2 * mDt_s, targetOmega_rad_Hz);
        }
        else if (mOmega_rad_Hz > targetOmega_rad_Hz)
        {
            mOmega_rad_Hz =
                std::max(mOmega_rad_Hz - mRampRateRef_rad_Hz2 * mDt_s, targetOmega_rad_Hz);
        }

        mTheta_rad += mOmega_rad_Hz * mDt_s;

        mTheta_rad = std::fmod(mTheta_rad, math::TWO_PI);

        if (mTheta_rad > math::PI)
        {
            mTheta_rad -= math::TWO_PI;
        }
        else if (mTheta_rad <= -math::PI)
        {
            mTheta_rad += math::TWO_PI;
        }
    }

    float getTheta_rad() const
    {
        return mTheta_rad;
    }

    float getOmega_rad_Hz() const
    {
        return mOmega_rad_Hz;
    }

    void reset()
    {
        mTheta_rad = 0.0f;
        mOmega_rad_Hz = 0.0f;
    }

  private:
    float mDt_s;
    const float& mRampRateRef_rad_Hz2;
    float mTheta_rad{0.0f};
    float mOmega_rad_Hz{0.0f};
};

} // namespace app