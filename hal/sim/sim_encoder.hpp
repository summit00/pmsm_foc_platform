#pragma once
#include "interfaces.hpp"
#include "motor_model.hpp"
#include <cmath>
#include <numbers>

namespace sim
{

class SimEncoder : public app::IEncoder
{
  public:
    explicit SimEncoder(const MotorModel& motor, uint32_t maxTicks, float polePairs)
        : mMotor(motor), mMaxTicks_count(maxTicks), mPolePairs_count(polePairs)
    {
    }

    uint16_t read_raw() const override
    {
        // currently inverse becasue in QEI_sensor is a hardcode inverse for the Physical sensor.
        // float processed_ticks = mMaxTicks_count - static_cast<float>(corrected_ticks);
        // TODO: fix in QEI_sensor and remove minus here.
        float thetaMech = -mMotor.getState().mThetaMech_rad;

        float normalizedTheta = thetaMech;
        if (normalizedTheta < 0.0f)
        {
            normalizedTheta += math::TWO_PI; //
        }

        float ticks = (normalizedTheta / math::TWO_PI) * mMaxTicks_count;

                return static_cast<uint16_t>(ticks) % mMaxTicks_count;
    }

    void reset() override
    {
        // Not strictly needed for the pure physics sim, but satisfies interface
    }

  private:
    const MotorModel& mMotor;
    uint32_t mMaxTicks_count;
    float mPolePairs_count;
};

} // namespace sim