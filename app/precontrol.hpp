#pragma once

#include <tuple>

namespace app
{

class Precontrol
{
  public:
    explicit Precontrol(const float& R_ohm,
                        const float& Ld_H,
                        const float& Lq_H,
                        const float& flux_pm_Wb)
        : R_ohm(R_ohm), Ld_H(Ld_H), Lq_H(Lq_H), flux_pm_Wb(flux_pm_Wb)
    {
    }

    std::tuple<float, float> compute(float id_ref, float iq_ref, float omega_rad_Hz) const
    {
        return {-omega_rad_Hz * Lq_H * iq_ref,
                omega_rad_Hz * Ld_H * id_ref + omega_rad_Hz * flux_pm_Wb};
    }

  private:
    const float& R_ohm;
    const float& Ld_H;
    const float& Lq_H;
    const float& flux_pm_Wb;
};

} // namespace app
