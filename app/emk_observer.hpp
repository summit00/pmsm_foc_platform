// emk_observer.hpp
#pragma once
#include "sensor.hpp"

namespace app
{

class EmkObserver : public ISensor
{
  public:
    EmkObserver(float dt_s,
                const float& uAlphaRef_V,
                const float& uBetaRef_V,
                const float& iAlphaRef_A,
                const float& iBetaRef_A)
        : mDt_s(dt_s), mUalphaRef_V(uAlphaRef_V), mUbetaRef_V(uBetaRef_V),
          mIalphaRef_A(iAlphaRef_A), mIbetaRef_A(iBetaRef_A)
    {
    }

    void update() override
    {
        // Observe back-EMF natively using mDt_s, mUalphaRef_V, mIalphaRef_A, etc.
        // Guarantee wrap around using the same fmod and pi checks here.
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
    }

  private:
    float mDt_s;
    const float& mUalphaRef_V;
    const float& mUbetaRef_V;
    const float& mIalphaRef_A;
    const float& mIbetaRef_A;

    float mTheta_rad{0.0f};
    float mOmega_rad_Hz{0.0f};
};

} // namespace app