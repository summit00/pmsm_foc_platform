#include <algorithm>

class ADRC
{
  public:
    ADRC(float samplingPeriod_s,
         float systemGain,
         float bandwidthController_rad_Hz,
         float bandwidthObserver_rad_Hz)
        : mSamplingPeriod_s(samplingPeriod_s), mSystemGain(systemGain),
          mBandwidthController_rad_Hz(bandwidthController_rad_Hz),
          mBandwidthObserver_rad_Hz(bandwidthObserver_rad_Hz)
    {
        reset();
    }

    void reset()
    {
        z1 = 0.0f; // Estimated Speed
        z2 = 0.0f; // Estimated Disturbance
        u = 0.0f;  // Last output
    }

    float update(float target, float feedback, float limit)
    {
        // Observer Gains (Bandwidth Parameterization)
        const float beta1 = 2.0f * mBandwidthObserver_rad_Hz;
        const float beta2 = mBandwidthObserver_rad_Hz * mBandwidthObserver_rad_Hz;

        // 1. Calculate Observer Error
        float error_obs = feedback - z1;

        // 2. Update Extended State Observer (ESO)
        // z1: Speed estimate, z2: Disturbance estimate
        z1 += (z2 + beta1 * error_obs + mSystemGain * u) * mSamplingPeriod_s;
        z2 += (beta2 * error_obs) * mSamplingPeriod_s;

        // 3. State Error Feedback (Proportional Law)
        // Kp is equivalent to the controller bandwidth wc
        float u0 = mBandwidthController_rad_Hz * (target - z1);

        // 4. Disturbance Compensation and Scaling
        u = (u0 - z2) / mSystemGain;

        // 5. Apply Dynamic Saturation
        u = std::clamp(u, -limit, limit);

        return u;
    }

    // Setters for real-time tuning adjustments
    void set_wc(float wc)
    {
        mBandwidthController_rad_Hz = wc;
    }
    void set_wo(float wo)
    {
        mBandwidthObserver_rad_Hz = wo;
    }
    void set_b0(float b0)
    {
        mSystemGain = b0;
    }

  private:
    // Parameters
    float mSamplingPeriod_s; // Sampling period
    float mSystemGain;       // System gain
    float mBandwidthController_rad_Hz;
    float mBandwidthObserver_rad_Hz;

    // States
    float z1 = 0.0f;
    float z2 = 0.0f;
    float u = 0.0f;
};
