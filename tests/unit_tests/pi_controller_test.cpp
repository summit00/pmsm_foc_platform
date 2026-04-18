#include "pi_controller.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

TEST_CASE("PI controller test")
{
    PIController pi;
    pi.setGains(1.0f, 0.5f);

    const float min_lim = -10.0f;
    const float max_lim = 10.0f;

    SECTION("capture 3 steps of correct integration within bounds")
    {
        // P = 1*2=2, I = 0.5*2=1. Total = 3
        CHECK(pi.compute(2.0f, 0.0f, min_lim, max_lim) == 3.0f);

        // P = 2, I = 1 + (0.5*2)=2. Total = 4
        CHECK(pi.compute(2.0f, 0.0f, min_lim, max_lim) == 4.0f);

        // P = 1*1=1, I = 2 + (0.5*1)=2.5. Total = 3.5
        CHECK(pi.compute(2.0f, 1.0f, min_lim, max_lim) == 3.5f);
    }

    SECTION("saturation and anti-windup clamping")
    {
        pi.reset();

        // P = 20, I = 10. Total = 30 (Exceeds limit 10)
        float out1 = pi.compute(20.0f, 0.0f, min_lim, max_lim);
        CHECK(out1 == 10.0f);

        // P = 20, I = 10. Total = 30 (Exceeds limit 10)
        float out2 = pi.compute(20.0f, 0.0f, min_lim, max_lim);
        CHECK(out2 == 10.0f);

        // P = -5, I = 10 + -5 * 0,5 = 7,5. Total = 30 (Exceeds limit 10)
        float out_recovery = pi.compute(0.0f, 5.0f, min_lim, max_lim);
        CHECK(out_recovery == 2.5f);
    }

    SECTION("dynamic limit change")
    {
        pi.reset();
        // P = 5, I = 0,5*5 = 2,5. Total = 7,5
        pi.compute(5.0f, 0.0f, -5.0f, 5.0f);
        // P = 5, I = 2,5+0,5*5 = 5. Total = 10
        CHECK(pi.compute(5.0f, 0.0f, -20.0f, 20.0f) == 10.0f);
    }
}
