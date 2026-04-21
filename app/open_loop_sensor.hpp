// open_loop_sensor.hpp
#pragma once
#include "math.hpp"
#include "sensor.hpp"
#include <cmath>

namespace app
{

class OpenLoopSensor : public ISensor
{
  public:
    OpenLoopSensor(float dt_s,
                   const float& currentOmegaRef_rad_Hz,
                   const bool& motorEnabledRef_bool)
        : mDt_s(dt_s), mOmegaRef(currentOmegaRef_rad_Hz),
          mMotorEnabledRef_bool(motorEnabledRef_bool)
    {
    }

    void update() override
    {
        if (mMotorEnabledRef_bool)
        {
            // Directly integrate the reference omega to find theta
            mTheta_rad += mOmegaRef * mDt_s;

            // Wrap angle between -PI and PI
            if (mTheta_rad > math::PI)
                mTheta_rad -= math::TWO_PI;
            if (mTheta_rad <= -math::PI)
                mTheta_rad += math::TWO_PI;
        }
        else
        {
            mTheta_rad = 0.0f;
        }
    }

    float getTheta_rad() const override
    {
        return mTheta_rad;
    }

    float getOmega_rad_Hz() const override
    {
        return mOmegaRef;
    }

    void reset() override
    {
        mTheta_rad = 0.0f;
    }

  private:
    float mDt_s;
    const float& mOmegaRef;
    const bool& mMotorEnabledRef_bool;
    float mTheta_rad{0.0f};
};

} // namespace app