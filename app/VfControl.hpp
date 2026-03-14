// #pragma once
// #include "math.hpp"
// #include <cmath>
// #include <tuple>

// namespace app
// {

// class VfControl
// {
//   public:
//     // v_per_hz: (Rated Voltage / Rated Frequency)
//     // v_boost: Voltage at 0 Hz to overcome R (e.g., 0.1V - 0.5V)
//     explicit VfControl(float v_per_hz, float v_boost)
//         : v_per_hz_raw(v_per_hz / (2.0f * 3.14159265f)), v_boost(v_boost)
//     {
//     }

//     // Input: Current electrical omega (from your ThetaGenerator) and theta
//     // Output: Normalized voltages for your PWM (assuming -1.0 to 1.0 range or Volts)
//     std::tuple<float, float, float> update(float omega_rad_s, float theta_rad)
//     {
//         // 1. V/f Law: Calculate magnitude (Vq)
//         // V = omega * (V/Hz factor) + Offset
//         float v_mag = std::abs(omega_rad_s) * v_per_hz_raw + v_boost;

//         // 2. Inverse Park Transform (assuming Vd = 0)
//         // Valpha = Vd*cos(theta) - Vq*sin(theta)
//         // Vbeta  = Vd*sin(theta) + Vq*cos(theta)
//         float v_alpha = -v_mag * math::sin(theta_rad);
//         float v_beta = v_mag * math::cos(theta_rad);

//         // 3. Inverse Clarke Transform (Standard)
//         float a = v_alpha;
//         float b = (-0.5f * v_alpha) + (0.8660254f * v_beta);
//         float c = (-0.5f * v_alpha) - (0.8660254f * v_beta);

//         return {a, b, c};
//     }

//   private:
//     float v_per_hz_raw; // V per rad/s
//     float v_boost;      // Static voltage offset
// };

// } // namespace app

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
