#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numbers>
#include <tuple>

#include "transform.hpp"

TEST_CASE("clarke transform")
{
    const float ia = 3.0f, ib = 2.0f, ic = -5.0f;
    CHECK(ia + ib + ic == Catch::Approx(0.0f));

    auto [alpha, beta] = Transforms::clarke(ia, ic);
    CHECK(alpha == Catch::Approx(ia));
    CHECK(beta ==
          Catch::Approx(ia * std::numbers::inv_sqrt3 + ib * 2.0f * std::numbers::inv_sqrt3));
}

TEST_CASE("inverse clarke transform")
{
    float alpha = 1.0f;
    float beta = 0.0f;

    auto [a, b, c] = Transforms::inverseClarke(alpha, beta);

    REQUIRE(a == Catch::Approx(1.0f));
    REQUIRE(b == Catch::Approx(-0.5f));
    REQUIRE(c == Catch::Approx(-0.5f));
}

TEST_CASE("park transform")
{
    constexpr float theta = std::numbers::pi_v<float> / 2.0f;

    float alpha = 1.0f;
    float beta = 0.0f;

    auto [d, q] = Transforms::park(alpha, beta, theta);

    REQUIRE(d == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(q == Catch::Approx(-1.0f).margin(1e-6f));
}

TEST_CASE("inverse park transform")
{
    constexpr float theta = std::numbers::pi_v<float> / 3.0f;

    float alpha = 0.8f;
    float beta = -0.4f;

    auto [d, q] = Transforms::park(alpha, beta, theta);
    auto [alpha2, beta2] = Transforms::inversePark(d, q, theta);

    REQUIRE(alpha2 == Catch::Approx(alpha).margin(1e-5f));
    REQUIRE(beta2 == Catch::Approx(beta).margin(1e-5f));
}