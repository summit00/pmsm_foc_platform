#pragma once
#include "encoder.hpp"
#include "interfaces.hpp"
#include "math.hpp"
#include "theta_generator.hpp"

namespace app
{
class Sensor
{
  public:
    Sensor(IEncoder& encoder, float polePairs) : encoder(encoder), mPolePairs(polePairs)
    {
    }

    void runSensors(float maxEncoderTicks, float targetOmega_rad_Hz, bool motor_enabled)
    {
        int32_t current_raw = encoder.read_raw();

        int32_t ticks_int = static_cast<int32_t>(maxEncoderTicks);
        int32_t raw_diff = current_raw - mQEIOffset_ticks;

        int32_t corrected_counts = (raw_diff % ticks_int + ticks_int) % ticks_int;

        float processed_counts = maxEncoderTicks - static_cast<float>(corrected_counts);
        rad_from_ticks(processed_counts, maxEncoderTicks);

        calculateOmegaEncoder_rad_Hz();

        if (motor_enabled)
        {
            theta_generator.update(targetOmega_rad_Hz);
        }
        else
        {
            theta_generator.reset();
        }
        mTheta_OpenLoop_rad = theta_generator.get_theta_rad();
    }

    void rad_from_ticks(float encoderTicks, float maxEncoderTicks)
    {
        float rawTheta = (encoderTicks / maxEncoderTicks) * math::TWO_PI * mPolePairs;

        mTheta_Encoder_rad = fmodf(rawTheta, math::TWO_PI);

        if (mTheta_Encoder_rad < 0)
        {
            mTheta_Encoder_rad += math::TWO_PI;
        }
    }

    float getThetaEncoder() const
    {
        return mTheta_Encoder_rad;
    }

    float getThetaOpenLoop() const
    {
        return mTheta_OpenLoop_rad;
    }

    float mech_rpm_to_elec_rad_per_sec(int16_t rpm, float polePairs) const
    {
        constexpr float rpm_to_rad_Hz = math::PI / 30.0f;
        return static_cast<float>(rpm) * polePairs * rpm_to_rad_Hz;
    }

    float getOmegaOpenLoop_rad_Hz() const
    {
        return theta_generator.get_omega_rad_Hz();
    }

    void calculateOmegaEncoder_rad_Hz()
    {
        // 1. Calculate raw change (Delta Theta)
        float deltaTheta = mTheta_Encoder_rad - mTheta_Encoder_rad_old;

        // 2. Handle the "Wrap-Around" jump (Crucial!)
        // If moving from 6.27 rad to 0.01 rad, delta is -6.26.
        // We must correct this to ~0.02.
        if (deltaTheta > math::PI)
            deltaTheta -= math::TWO_PI;
        if (deltaTheta < -math::PI)
            deltaTheta += math::TWO_PI;

        // 3. Calculate raw velocity (rad/s)
        float rawOmega = deltaTheta * 20000.0f;

        // 4. Apply IIR Filter: y[n] = α * x[n] + (1 - α) * y[n-1]
        mOmega_Encoder_rad_Hz =
            (mFilterAlpha * rawOmega) + ((1.0f - mFilterAlpha) * mOmega_Encoder_rad_Hz);

        // 5. Update old value for next cycle
        mTheta_Encoder_rad_old = mTheta_Encoder_rad;
    }

    float getOmegaEncoder_rad_Hz() const
    {
        return mOmega_Encoder_rad_Hz;
    }

  private:
    IEncoder& encoder;
    ThetaGenerator theta_generator{1.0f / 20000.0f, mech_rpm_to_elec_rad_per_sec(500, 4.0f)};
    float mTheta_Encoder_rad{0.0f};
    float mTheta_Encoder_rad_old{0.0f};
    float mTheta_OpenLoop_rad{0.0f};
    float mOmega_Encoder_rad_Hz{0.0f};
    float mPolePairs{4.0f};
    float mFilterAlpha = 0.01f;
    uint16_t mQEIOffset_ticks{295};
};
} // namespace app