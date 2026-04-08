#pragma once
#include "math.hpp"
#include "transform.hpp"
#include <cmath>
#include <tuple>

namespace app
{

class VfControl
{
  public:
    // v_per_hz: (Rated Voltage / Rated Frequency).
    // v_boost: Voltage at 0 Hz to overcome R (e.g., 0.1V - 0.5V).
    explicit VfControl(float v_per_hz, float v_boost)
        : v_per_hz_raw(v_per_hz / (2.0f * math::PI)), v_boost(v_boost)
    {
    }

    std::tuple<float, float, float> update(float omega_rad_s, float theta_rad)
    {
        // V/f Law: Calculate magnitude (Vq)
        // V = omega * (V/Hz factor) + Offset
        float v_mag = std::abs(omega_rad_s) * v_per_hz_raw + v_boost;

        auto [v_alpha, v_beta] = Transforms::inversePark(0.0f, v_mag, theta_rad);

        auto [a, b, c] = Transforms::inverseClarke(v_alpha, v_beta);

        return {a, b, c};
    }

  private:
    float v_per_hz_raw; // V per rad/s
    float v_boost;      // Static voltage offset
};

} // namespace app
