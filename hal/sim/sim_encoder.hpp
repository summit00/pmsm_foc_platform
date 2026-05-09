// sim_encoder.hpp
namespace sim
{
class SimEncoder : public app::IEncoder
{
  public:
    explicit SimEncoder(const MotorModel& motor, uint32_t maxTicks, float polePairs)
        : mMotor(motor), mMaxTicks_count(maxTicks), mPolePairs_count(polePairs)
    {
    }

    // New method to set the alignment offset
    void set_alignment_offset(float offset_rad)
    {
        mOffset_rad = offset_rad;
    }

    uint16_t read_raw() const override
    {
        float thetaMech = -mMotor.getState().mThetaMech_rad + mOffset_rad;

        float normalizedTheta = std::fmod(thetaMech, math::TWO_PI);
        if (normalizedTheta < 0.0f)
        {
            normalizedTheta += math::TWO_PI;
        }

        float ticks = (normalizedTheta / math::TWO_PI) * mMaxTicks_count;

        return static_cast<uint16_t>(ticks) % mMaxTicks_count;
    }

    void reset() override
    {
    }

  private:
    const MotorModel& mMotor;
    uint32_t mMaxTicks_count;
    float mPolePairs_count;
    float mOffset_rad{0.0f}; // Default to no offset
};
} // namespace sim