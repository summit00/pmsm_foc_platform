#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <numbers>

#include "transform.hpp"


TEST_CASE("clarke transform")
{
    const float ia = 3.0f, ib = 2.0f, ic = -5.0f;
    CHECK(ia + ib + ic == Catch::Approx(0.0f));

    auto [alpha, beta] = Transforms::clarke({ia, ib, ic});
    CHECK(alpha == Catch::Approx(ia));
    CHECK(beta == Catch::Approx(ia*std::numbers::inv_sqrt3 + ib*2.0f*std::numbers::inv_sqrt3));
}

TEST_CASE("inverse clarke transform")
{
    std::array<float,2> ab{1.0f, 0.0f};

    auto abc = Transforms::inverseClarke(ab);

    REQUIRE(abc[0] == Catch::Approx(1.0f));
    REQUIRE(abc[1] == Catch::Approx(-0.5f));
    REQUIRE(abc[2] == Catch::Approx(-0.5f));
}

TEST_CASE("park transform")
{
    constexpr float theta = std::numbers::pi_v<float> / 2.0f;

    std::array<float,2> ab{1.0f, 0.0f};

    auto dq = Transforms::park(ab, theta);

    REQUIRE(dq[0] == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(dq[1] == Catch::Approx(-1.0f).margin(1e-6f));
}

TEST_CASE("inverse park transform")
{
    constexpr float theta = std::numbers::pi_v<float> / 3.0f;

    std::array<float,2> ab{0.8f, -0.4f};

    auto dq  = Transforms::park(ab, theta);
    auto ab2 = Transforms::inversePark(dq, theta);

    REQUIRE(ab2[0] == Catch::Approx(ab[0]).margin(1e-5f));
    REQUIRE(ab2[1] == Catch::Approx(ab[1]).margin(1e-5f));
}