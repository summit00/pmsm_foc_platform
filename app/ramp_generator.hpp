#pragma once
#include <algorithm>

namespace app
{

class RampGenerator
{
  public:
    explicit RampGenerator(float pwmPeriod_s) : mPwmPeriod_s(pwmPeriod_s)
    {
    }

    float update(float target, float acceleration)
    {
        float maxStep = acceleration * mPwmPeriod_s;

        if (mCurrentValue < target)
        {
            mCurrentValue = std::min(mCurrentValue + maxStep, target);
        }
        else if (mCurrentValue > target)
        {
            mCurrentValue = std::max(mCurrentValue - maxStep, target);
        }

        return mCurrentValue;
    }

    void reset(float value = 0.0f)
    {
        mCurrentValue = value;
    }

    float getValue() const
    {
        return mCurrentValue;
    }

  private:
    float mPwmPeriod_s;
    float mCurrentValue{0.0f};
};

} // namespace app