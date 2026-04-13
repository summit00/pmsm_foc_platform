#include "math.hpp"
#include <algorithm>
#include <array>

class PIController
{
  public:
    PIController() = default;

    float compute(float setpoint, float measured, float min, float max)
    {
        float error = setpoint - measured;
        float prop = mKp * error;

        mIntegral = std::clamp(mIntegral + mKi * prop, min, max);

        return std::clamp(prop + mIntegral, min, max);
    }

    void setGains(float kp, float ki)
    {
        mKp = kp;
        mKi = ki;
        if (mKi == 0.0f)
        {
            reset();
        }
    }

    void getGains(float& kp, float& ki) const
    {
        kp = mKp;
        ki = mKi;
    }

    void reset()
    {
        mIntegral = 0.0f;
    }

    std::array<float, 2> calculatePIGains(float R, float L, float Ts)
    {
        float Kp = L / (2.0f * Ts);
        float Ki = L / R;

        return {Kp, Ki};
    }

  private:
    float mKp;
    float mKi;
    float mIntegral = 0.0f;
};