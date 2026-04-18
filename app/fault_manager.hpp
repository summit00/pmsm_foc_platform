#pragma once
#include <cmath>
#include <cstdint>

namespace app
{

class FaultManager
{
  public:
    enum class FaultType : uint8_t
    {
        NONE = 0,
        OVERCURRENT = 1,
        OVERVOLTAGE = 4,
        UNDERVOLTAGE = 8,
        OVERTEMP = 16,
    };

    void checkForFaults(float ia_A, float ic_A, float dcBus_V, float temp_C)
    {

        uint8_t newFaults = 0;

        float ib_A = -ia_A - ic_A;
        if (std::abs(ia_A) > mOverCurrentThreshold_A || std::abs(ib_A) > mOverCurrentThreshold_A ||
            std::abs(ic_A) > mOverCurrentThreshold_A)
        {
            newFaults |= static_cast<uint8_t>(FaultType::OVERCURRENT);
        }

        if (dcBus_V > 48.0f)
            newFaults |= static_cast<uint8_t>(FaultType::OVERVOLTAGE);
        if (dcBus_V < 10.0f)
            newFaults |= static_cast<uint8_t>(FaultType::UNDERVOLTAGE);
        if (temp_C > 80.0f)
            newFaults |= static_cast<uint8_t>(FaultType::OVERTEMP);

        mCurrentFault = static_cast<FaultType>(newFaults);
    }

    bool isFaulted() const
    {
        return mCurrentFault != FaultType::NONE;
    }

    uint8_t getFaultType() const
    {
        return static_cast<uint8_t>(mCurrentFault);
    }

    void clearFaults()
    {
        mCurrentFault = FaultType::NONE;
    }

  private:
    float mOverCurrentThreshold_A{20.7f};
    FaultType mCurrentFault{FaultType::NONE};
};

} // namespace app