#pragma once
#include <cmath>
#include <numbers>

namespace app
{

class ThetaGenerator
{
  public:
    explicit ThetaGenerator(float dt_s, float ramp_rate_rad_Hz2)
        : dt_s(dt_s), ramp_rate_rad_Hz2(ramp_rate_rad_Hz2)
    {
    }

    float update(float target_omega_rad_Hz)
    {
        // Ramp omega toward target.
        if (omega_rad_Hz < target_omega_rad_Hz)
            omega_rad_Hz = std::min(omega_rad_Hz + ramp_rate_rad_Hz2 * dt_s, target_omega_rad_Hz);
        else if (omega_rad_Hz > target_omega_rad_Hz)
            omega_rad_Hz = std::max(omega_rad_Hz - ramp_rate_rad_Hz2 * dt_s, target_omega_rad_Hz);

        theta_rad += omega_rad_Hz * dt_s;

        // Wrap to [0, 2π].
        if (theta_rad > 2.0f * std::numbers::pi_v<float>)
            theta_rad -= 2.0f * std::numbers::pi_v<float>;
        else if (theta_rad < 0.0f)
            theta_rad += 2.0f * std::numbers::pi_v<float>;

        return theta_rad;
    }

    float get_theta_rad() const
    {
        return theta_rad;
    }
    float get_omega_rad_Hz() const
    {
        return omega_rad_Hz;
    }

    void reset()
    {
        theta_rad = 0.0f;
        omega_rad_Hz = 0.0f;
    }

  private:
    float dt_s;
    float ramp_rate_rad_Hz2;
    float theta_rad = 0.0f;
    float omega_rad_Hz = 0.0f;
};

} // namespace app