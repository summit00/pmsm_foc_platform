#include "dq_limiter.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("DQ Limiter with saturation")
{

    SECTION("no saturation when magnitude is within limit")
    {
        // mag_ref = sqrt(3^2 + 4^2) = 5, x_max = 10
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(3.0f, 4.0f, 10.0f);

        CHECK(d_sat == Catch::Approx(3.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(4.0f).epsilon(1e-5f));
    }

    SECTION("no saturation at exact limit")
    {
        // mag_ref = sqrt(6^2 + 8^2) = 10, x_max = 10
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(6.0f, 8.0f, 10.0f);

        CHECK(d_sat == Catch::Approx(6.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(8.0f).epsilon(1e-5f));
    }

    SECTION("saturation when magnitude exceeds limit")
    {
        // mag_ref = sqrt(6^2 + 8^2) = 10, x_max = 5
        // scale = 5 / 10 = 0.5
        // d_sat = 6 * 0.5 = 3, q_sat = 8 * 0.5 = 4
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(6.0f, 8.0f, 5.0f);

        CHECK(d_sat == Catch::Approx(3.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(4.0f).epsilon(1e-5f));
    }

    SECTION("saturation with equal d and q components")
    {
        // mag_ref = sqrt(5^2 + 5^2) = sqrt(50) ≈ 7.071, x_max = 5
        // scale = 5 / 7.071 ≈ 0.7071
        // d_sat = q_sat = 5 * 0.7071 ≈ 3.536
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(5.0f, 5.0f, 5.0f);

        float expected = 5.0f * (5.0f / std::sqrt(50.0f));
        CHECK(d_sat == Catch::Approx(expected).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(expected).epsilon(1e-5f));
    }

    SECTION("negative components within limit")
    {
        // mag_ref = sqrt((-3)^2 + (-4)^2) = 5, x_max = 10
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(-3.0f, -4.0f, 10.0f);

        CHECK(d_sat == Catch::Approx(-3.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(-4.0f).epsilon(1e-5f));
    }

    SECTION("negative components with saturation")
    {
        // mag_ref = sqrt((-6)^2 + (-8)^2) = 10, x_max = 5
        // scale = 5 / 10 = 0.5
        // d_sat = -6 * 0.5 = -3, q_sat = -8 * 0.5 = -4
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(-6.0f, -8.0f, 5.0f);

        CHECK(d_sat == Catch::Approx(-3.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(-4.0f).epsilon(1e-5f));
    }

    SECTION("mixed sign components with saturation")
    {
        // mag_ref = sqrt(6^2 + (-8)^2) = 10, x_max = 7.5
        // scale = 7.5 / 10 = 0.75
        // d_sat = 6 * 0.75 = 4.5, q_sat = -8 * 0.75 = -6
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(6.0f, -8.0f, 7.5f);

        CHECK(d_sat == Catch::Approx(4.5f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(-6.0f).epsilon(1e-5f));
    }

    SECTION("zero components")
    {
        // mag_ref = 0, x_max = 10
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(0.0f, 0.0f, 10.0f);

        CHECK(d_sat == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(0.0f).epsilon(1e-5f));
    }

    SECTION("only d component non-zero within limit")
    {
        // mag_ref = 5, x_max = 10
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(5.0f, 0.0f, 10.0f);

        CHECK(d_sat == Catch::Approx(5.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(0.0f).epsilon(1e-5f));
    }

    SECTION("only d component non-zero with saturation")
    {
        // mag_ref = 15, x_max = 5
        // scale = 5 / 15 = 1/3
        // d_sat = 15 * (1/3) = 5, q_sat = 0
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(15.0f, 0.0f, 5.0f);

        CHECK(d_sat == Catch::Approx(5.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(0.0f).epsilon(1e-5f));
    }

    SECTION("only q component non-zero with saturation")
    {
        // mag_ref = 20, x_max = 8
        // scale = 8 / 20 = 0.4
        // d_sat = 0, q_sat = 20 * 0.4 = 8
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(0.0f, 20.0f, 8.0f);

        CHECK(d_sat == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(8.0f).epsilon(1e-5f));
    }

    SECTION("very small limit value")
    {
        // mag_ref = sqrt(3^2 + 4^2) = 5, x_max = 0.1
        // scale = 0.1 / 5 = 0.02
        // d_sat = 3 * 0.02 = 0.06, q_sat = 4 * 0.02 = 0.08
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(3.0f, 4.0f, 0.1f);

        CHECK(d_sat == Catch::Approx(0.06f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(0.08f).epsilon(1e-5f));
    }

    SECTION("large magnitude with saturation")
    {
        // mag_ref = sqrt(300^2 + 400^2) = 500, x_max = 250
        // scale = 250 / 500 = 0.5
        // d_sat = 300 * 0.5 = 150, q_sat = 400 * 0.5 = 200
        auto [d_sat, q_sat] = app::DQLimiter::applyLimit(300.0f, 400.0f, 250.0f);

        CHECK(d_sat == Catch::Approx(150.0f).epsilon(1e-5f));
        CHECK(q_sat == Catch::Approx(200.0f).epsilon(1e-5f));
    }
}