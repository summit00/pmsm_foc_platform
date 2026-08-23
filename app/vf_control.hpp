#pragma once
#include "math.hpp"
#include <tuple>
#include <cmath>

namespace app
{

class VfControl
{
  public:
    // psi: Permanent magnet flux linkage (flux_pm_Wb) in Weber.
    // v_boost: Voltage at 0 Hz to overcome R (e.g., 0.1V - 0.5V).
    explicit VfControl(float psi, float v_boost)
        : v_boost(v_boost)
    {
        // Rated voltage per Hz (V/Hz) = Psi * 2 * PI
        float v_per_hz = psi * (2.0f * math::PI);
        // Raw V/rad/s factor = (V/Hz) / (2 * PI) = Psi
        v_per_hz_raw = v_per_hz / (2.0f * math::PI);
    }

    std::tuple<float, float> update(float omega_rad_s)
    {
        // V/f Law: Calculate magnitude (Vq)
        // V = omega * (V/Hz factor) + Offset
        float v_mag = std::abs(omega_rad_s) * v_per_hz_raw + v_boost;
        return {0.0f, v_mag};
    }

  private:
    float v_per_hz_raw; // V per rad/s
    float v_boost;      // Static voltage offset
};

} // namespace app
