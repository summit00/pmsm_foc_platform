#pragma once

#include "motor_params.hpp"
#include <tuple>

namespace app
{

class Precontrol
{
  public:
    explicit Precontrol(const MotorParams& params) : mParams(params)
    {
    }

    std::tuple<float, float> compute(float id_ref, float iq_ref, float omega_rad_Hz) const
    {
        return {-omega_rad_Hz * mParams.Lq_H * iq_ref,
                omega_rad_Hz * mParams.Ld_H * id_ref + omega_rad_Hz * mParams.flux_pm_Wb};
    }

  private:
    const MotorParams& mParams;
};

} // namespace app
