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
    };

    explicit FaultManager(float overCurrentThreshold_A)
        : mOverCurrentThreshold_A(overCurrentThreshold_A)
    {
    }

    void checkForFaults(float ia_A, float ic_A)
    {
        if (mCurrentFault != FaultType::NONE)
        {
            return;
        }

        float ib_A = -ia_A - ic_A;

        if (std::abs(ia_A) > mOverCurrentThreshold_A || std::abs(ib_A) > mOverCurrentThreshold_A ||
            std::abs(ic_A) > mOverCurrentThreshold_A)
        {
            mCurrentFault = FaultType::OVERCURRENT;
        }
    }

    bool isFaulted() const
    {
        return mCurrentFault != FaultType::NONE;
    }

    FaultType getFaultType() const
    {
        return mCurrentFault;
    }

    void clearFaults()
    {
        mCurrentFault = FaultType::NONE;
    }

  private:
    float mOverCurrentThreshold_A;
    FaultType mCurrentFault{FaultType::NONE};
};

} // namespace app