#pragma once
#include <cmath>
#include <numbers>
#include <tuple>

namespace math
{

constexpr float PI = std::numbers::pi_v<float>;
constexpr float INV_PI = 1.0f / PI;
constexpr float INV_SQRT_3 = std::numbers::inv_sqrt3_v<float>;
constexpr float SQRT_3 = std::numbers::sqrt3_v<float>;

inline float sin_poly(float x) noexcept
{
    float u = 1.3528548e-10f;
    u = u * x + -2.4703144e-08f;
    u = u * x + 2.7532926e-06f;
    u = u * x + -0.00019840381f;
    u = u * x + 0.0083333179f;
    return u * x + -0.16666666f;
}

inline float cos_poly(float x) noexcept
{
    float u = 1.7290616e-09f;
    u = u * x + -2.7093486e-07f;
    u = u * x + 2.4771643e-05f;
    u = u * x + -0.0013887906f;
    u = u * x + 0.041666519f;
    return u * x + -0.49999991f;
}

inline float reduce(float& x) noexcept
{
    int si = static_cast<int>(x * INV_PI);
    x = x - static_cast<float>(si) * PI;

    if (si & 1)
    {
        x = x > 0.0f ? x - PI : x + PI;
    }

    return x;
}

inline float square(float x) noexcept
{
    return x * x;
}

inline float sin(float x) noexcept
{
    reduce(x);
    return x + x * x * x * sin_poly(x * x);
}

inline float cos(float x) noexcept
{
    reduce(x);
    return 1.0f + x * x * cos_poly(x * x);
}

inline std::tuple<float, float> sin_cos(float x)
{
    reduce(x);
    float x2 = x * x;
    float sin_x = x + x * x2 * sin_poly(x2);
    float cos_x = 1.0f + x2 * cos_poly(x2);
    return {sin_x, cos_x};
}

} // namespace math