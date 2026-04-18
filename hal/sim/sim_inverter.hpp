#pragma once
#include "interfaces.hpp"

namespace sim
{

class SimInverter : public app::IInverter
{
  public:
    void
    set_phase_voltages(float va_V, float vb_V, float vc_V, float vbus_V, bool isEnabled) override
    {
        if (isEnabled)
        {
            mVu_V = va_V;
            mVv_V = vb_V;
            mVw_V = vc_V;
        }
        else
        {
            mVu_V = 0.0f;
            mVv_V = 0.0f;
            mVw_V = 0.0f;
        }
    }

    float getVu() const
    {
        return mVu_V;
    }
    float getVv() const
    {
        return mVv_V;
    }
    float getVw() const
    {
        return mVw_V;
    }

  private:
    float mVu_V{0.0f};
    float mVv_V{0.0f};
    float mVw_V{0.0f};
};

} // namespace sim