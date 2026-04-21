// encoder_sensor.hpp
#pragma once
#include "interfaces.hpp"
#include "math.hpp"
#include "sensor.hpp"
#include <cmath>
#include <numbers>

namespace app
{

class EncoderSensor : public ISensor
{
  public:
    EncoderSensor(
        IEncoder& encoder, float dt_s, float polePairs, float maxEncoderTicks, uint16_t offsetTicks)
        : mEncoder(encoder), mDt_s(dt_s), mPolePairs_count(polePairs),
          mMaxTicks_count(maxEncoderTicks), mOffset_ticks(offsetTicks)
    {
    }

    void update() override
    {
        readAndProcessAngle();
        calculateVelocity();
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
        mThetaOld_rad = 0.0f;
        mOmega_rad_Hz = 0.0f;
    }

    void setFilterAlpha(float alpha)
    {
        mFilterAlpha_coeff = alpha;
    }

  private:
    void readAndProcessAngle()
    {
        int32_t currentRaw_ticks = mEncoder.read_raw();
        int32_t maxTicksInt_ticks = static_cast<int32_t>(mMaxTicks_count);

        int32_t rawDiff_ticks = currentRaw_ticks - mOffset_ticks;
        int32_t corrected_ticks =
            (rawDiff_ticks % maxTicksInt_ticks + maxTicksInt_ticks) % maxTicksInt_ticks;

        float processed_ticks = mMaxTicks_count - static_cast<float>(corrected_ticks);

        float rawTheta_rad = (processed_ticks / mMaxTicks_count) * math::TWO_PI * mPolePairs_count;

        mTheta_rad = std::fmod(rawTheta_rad, math::TWO_PI);

        if (mTheta_rad > math::PI)
        {
            mTheta_rad -= math::TWO_PI;
        }
        else if (mTheta_rad <= -math::PI)
        {
            mTheta_rad += math::TWO_PI;
        }
    }

    void calculateVelocity()
    {
        float deltaTheta_rad = mTheta_rad - mThetaOld_rad;

        if (deltaTheta_rad > math::PI)
        {
            deltaTheta_rad -= math::TWO_PI;
        }
        else if (deltaTheta_rad < -math::PI)
        {
            deltaTheta_rad += math::TWO_PI;
        }

        float rawOmega_rad_Hz = deltaTheta_rad / mDt_s;

        mOmega_rad_Hz =
            (mFilterAlpha_coeff * rawOmega_rad_Hz) + ((1.0f - mFilterAlpha_coeff) * mOmega_rad_Hz);

        mThetaOld_rad = mTheta_rad;
    }

    IEncoder& mEncoder;
    float mDt_s;
    float mPolePairs_count;
    float mMaxTicks_count;
    uint16_t mOffset_ticks;

    float mTheta_rad{0.0f};
    float mThetaOld_rad{0.0f};
    float mOmega_rad_Hz{0.0f};
    float mFilterAlpha_coeff{0.05f};
};

} // namespace app