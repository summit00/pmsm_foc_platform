#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "transform.hpp"

using Catch::Matchers::WithinRel;

static bool near(float a, float b, float eps=1e-4f)
{
    return std::fabs(a - b) <= eps;
}

TEST_CASE("Clarke Transform")
{
    const float ia = 1.0f, ib = -0.5f, ic = -0.5f;
    auto ab = Transforms::clarke({ia, ib, ic});
    REQUIRE(near(ab[0], ia));
    REQUIRE(near(ab[1], 0.0f));
}

TEST_CASE("Inverse Clarke Transform")
{
    const float alpha = 1.0f, beta = 0.1f;
    auto abc = Transforms::inverseClarke({alpha, beta});
    REQUIRE(near(abc[0], alpha));
    REQUIRE(near(abc[1], -0.5f * alpha + 0.8660254f * beta));
    REQUIRE(near(abc[2], -0.5f * alpha - 0.8660254f * beta));
}