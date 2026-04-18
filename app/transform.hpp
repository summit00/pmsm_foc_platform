#pragma once
#include "math.hpp"
#include <cmath>
#include <numbers>
#include <tuple>

class Transforms
{
  public:
    static std::tuple<float, float> clarke(float a, float c)
    {
        float alpha = a;
        // Derived from Ib = -(Ia + Ic):
        // beta = (Ia + 2*(-Ia - Ic)) / sqrt(3) => (-Ia - 2Ic) / sqrt(3)
        float beta = (-a - 2.0f * c) * math::INV_SQRT_3;
        return {alpha, beta};
    }

    static std::tuple<float, float> clarke(float a, float b, [[maybe_unused]] float c)
    {
        float alpha = a;
        float beta = (a + 2.0f * b) * math::INV_SQRT_3;
        return {alpha, beta};
    }

    static std::tuple<float, float, float> inverseClarke(float alpha, float beta)
    {
        constexpr float sqrt3_over_2 = math::SQRT_3 / 2.0f;

        float a = alpha;
        float b = -0.5f * alpha + sqrt3_over_2 * beta;
        float c = -0.5f * alpha - sqrt3_over_2 * beta;
        return {a, b, c};
    }

    static std::tuple<float, float> park(float alpha, float beta, float theta)
    {
        float sin_t, cos_t;
        std::tie(sin_t, cos_t) = math::sin_cos(theta);
        float d = alpha * cos_t + beta * sin_t;
        float q = -alpha * sin_t + beta * cos_t;
        return {d, q};
    }

    static std::tuple<float, float> inversePark(float d, float q, float theta)
    {
        float sin_t, cos_t;
        std::tie(sin_t, cos_t) = math::sin_cos(theta);
        float alpha = d * cos_t - q * sin_t;
        float beta = d * sin_t + q * cos_t;
        return {alpha, beta};
    }
};
