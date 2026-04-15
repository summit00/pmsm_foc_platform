// open_loop_sensor.hpp
#pragma once
#include "sensor.hpp"
#include "theta_generator.hpp"

namespace app
{

class OpenLoopSensor : public ISensor
{
  public:
    OpenLoopSensor(float dt_s,
                   const float& rampRateRef_rad_Hz2,
                   const float& targetOmegaRef_rad_Hz,
                   const bool& motorEnabledRef_bool)
        : mThetaGenerator(dt_s, rampRateRef_rad_Hz2), mTargetOmegaRef_rad_Hz(targetOmegaRef_rad_Hz),
          mMotorEnabledRef_bool(motorEnabledRef_bool)
    {
    }

    void update() override
    {
        if (mMotorEnabledRef_bool)
        {
            mThetaGenerator.update(mTargetOmegaRef_rad_Hz);
        }
        else
        {
            mThetaGenerator.reset();
        }
    }

    float getTheta_rad() const override
    {
        return mThetaGenerator.getTheta_rad();
    }

    float getOmega_rad_Hz() const override
    {
        return mThetaGenerator.getOmega_rad_Hz();
    }

    void reset() override
    {
        mThetaGenerator.reset();
    }

  private:
    ThetaGenerator mThetaGenerator;
    const float& mTargetOmegaRef_rad_Hz;
    const bool& mMotorEnabledRef_bool;
};

} // namespace app