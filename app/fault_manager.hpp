#pragma once
#include <cmath>
#include <cstdint>

namespace app
{

struct FaultThresholds
{
    float overcurrent_threshold_A{7.0f};
    float overvoltage_threshold_V{48.0f};
    float undervoltage_threshold_V{10.0f};
    float overtemp_threshold_C{80.0f};
};

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

    explicit FaultManager(const FaultThresholds& thresholds = {})
        : mThresholds(thresholds)
    {
    }

    void checkForFaults(float ia_A, float ic_A, float dcBus_V, float temp_C)
    {
        uint8_t newFaults = 0;

        float ib_A = -ia_A - ic_A;
        if (std::abs(ia_A) > mThresholds.overcurrent_threshold_A ||
            std::abs(ib_A) > mThresholds.overcurrent_threshold_A ||
            std::abs(ic_A) > mThresholds.overcurrent_threshold_A)
        {
            newFaults |= static_cast<uint8_t>(FaultType::OVERCURRENT);
        }

        if (dcBus_V > mThresholds.overvoltage_threshold_V)
            newFaults |= static_cast<uint8_t>(FaultType::OVERVOLTAGE);
        if (dcBus_V < mThresholds.undervoltage_threshold_V)
            newFaults |= static_cast<uint8_t>(FaultType::UNDERVOLTAGE);
        if (temp_C > mThresholds.overtemp_threshold_C)
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

    void setThresholds(const FaultThresholds& thresholds)
    {
        mThresholds = thresholds;
    }

    const FaultThresholds& getThresholds() const
    {
        return mThresholds;
    }

  private:
    FaultThresholds mThresholds{};
    FaultType mCurrentFault{FaultType::NONE};
};

} // namespace app