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
    EncoderSensor(IEncoder& encoder,
                  float pwmPeriod_s,
                  float polePairs,
                  float maxEncoderTicks,
                  uint16_t offsetTicks)
        : mEncoder(encoder), mPwmPeriod_s(pwmPeriod_s), mPolePairs_count(polePairs),
          mMaxTicks_count(maxEncoderTicks), mOffset_ticks(offsetTicks)
    {
    }

    void update() override
    {
        readAndProcessAngle();
        updatePLL();
    }

    float getTheta_rad() const override
    {
        return mTheta_rad;
    }

    float getOmega_rad_Hz() const override
    {
        return mEstimatedOmega_rad_Hz;
    }

    void reset() override
    {
        mTheta_rad = 0.0f;
        mEstimatedTheta_rad = 0.0f;
        mEstimatedOmega_rad_Hz = 0.0f;
        mPllIntegrator = 0.0f;
    }

    void setPllGains(float kp, float ki)
    {
        mPllKp = kp;
        mPllKi = ki;
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

    void updatePLL()
    {
        float thetaError = mTheta_rad - mEstimatedTheta_rad;

        if (thetaError > math::PI)
            thetaError -= math::TWO_PI;
        else if (thetaError < -math::PI)
            thetaError += math::TWO_PI;

        mPllIntegrator += mPllKi * thetaError * mPwmPeriod_s;

        mEstimatedOmega_rad_Hz = (mPllKp * thetaError) + mPllIntegrator;

        mEstimatedTheta_rad += mEstimatedOmega_rad_Hz * mPwmPeriod_s;

        if (mEstimatedTheta_rad > math::PI)
            mEstimatedTheta_rad -= math::TWO_PI;
        else if (mEstimatedTheta_rad < -math::PI)
            mEstimatedTheta_rad += math::TWO_PI;
    }

    IEncoder& mEncoder;
    const float mPwmPeriod_s;
    const float mPolePairs_count;
    const float mMaxTicks_count;
    const uint16_t mOffset_ticks;

    float mTheta_rad{0.0f};

    // PLL State Variables
    float mEstimatedTheta_rad{0.0f};
    float mEstimatedOmega_rad_Hz{0.0f};
    float mPllIntegrator{0.0f};

    float mPllKp{355.0f};
    float mPllKi{63000.0f};
};

} // namespace app