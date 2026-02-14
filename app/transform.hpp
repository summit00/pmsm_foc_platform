#pragma once
#include <cmath>
#include <array>

class Transforms {
public:
    static std::array<float,2> clarke(const std::array<float,3>& abc) {
        float alpha = abc[0];
        float beta  = (abc[0] + 2.0f * abc[1]) / 1.7320508f; // sqrt(3)
        return {alpha, beta};
    }

    static std::array<float,3> inverseClarke(const std::array<float,2>& ab) {
        float a = ab[0];
        float b = -0.5f * ab[0] + 0.8660254f * ab[1]; // cos/sin 60 deg
        float c = -0.5f * ab[0] - 0.8660254f * ab[1];
        return {a, b, c};
    }

    static std::array<float,2> park(const std::array<float,2>& ab, float theta) {
        float d = ab[0] * std::cos(theta) + ab[1] * std::sin(theta);
        float q = -ab[0] * std::sin(theta) + ab[1] * std::cos(theta);
        return {d, q};
    }

    static std::array<float,2> inversePark(const std::array<float,2>& dq, float theta) {
        float alpha = dq[0] * std::cos(theta) - dq[1] * std::sin(theta);
        float beta  = dq[0] * std::sin(theta) + dq[1] * std::cos(theta);
        return {alpha, beta};
    }
};
