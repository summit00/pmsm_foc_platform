#pragma once
#include "interfaces.hpp"

namespace sim
{

class SimGateEnable : public app::IEnableOutput
{
  public:
    void set_enable(bool enabled) override
    {
        mIsEnabled_bool = enabled;
    }

    bool isEnabled() const
    {
        return mIsEnabled_bool;
    }

  private:
    bool mIsEnabled_bool{false};
};

} // namespace sim