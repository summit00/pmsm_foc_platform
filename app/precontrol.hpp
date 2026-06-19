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

    std::tuple<float, float>
    compute(float id_ref, float iq_ref, float omega_rad_Hz, float UsLimit_V) const
    {
        auto Udff_V = -omega_rad_Hz * mParams.Lq_H * iq_ref;
        auto Uqff_V = omega_rad_Hz * mParams.Ld_H * id_ref + omega_rad_Hz * mParams.flux_pm_Wb;

        auto Uabs = std::abs(std::sqrt(math::square(Udff_V) + math::square(Uqff_V)));

        if (Uabs > UsLimit_V)
        {
            Udff_V *= UsLimit_V / Uabs;
            Uqff_V *= UsLimit_V / Uabs;
        }

        return {Udff_V, Uqff_V};
    }

  private:
    const MotorParams& mParams;
};

} // namespace app
