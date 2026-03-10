#include "svm.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Space Vector Modulation Min-Max method")
{
    SpaceVectorModulation svm;

    SECTION("balanced three-phase voltages")
    {
        // All phases equal -> offset = va, output should be zero
        auto [va_out, vb_out, vc_out] = svm.applyModulation(50.0f, 50.0f, 50.0f);

        CHECK(va_out == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(vb_out == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(vc_out == Catch::Approx(0.0f).epsilon(1e-5f));
    }

    SECTION("single phase at maximum")
    {
        // va=100, vb=-50, vc=-50 -> offset=25, scale=2/sqrt(3)
        auto [va_out, vb_out, vc_out] = svm.applyModulation(100.0f, -50.0f, -50.0f);

        float scale = 2.0f / std::sqrt(3.0f);
        float expected_va = scale * (100.0f - 25.0f);
        float expected_vb = scale * (-50.0f - 25.0f);
        float expected_vc = scale * (-50.0f - 25.0f);

        CHECK(va_out == Catch::Approx(expected_va).epsilon(1e-5f));
        CHECK(vb_out == Catch::Approx(expected_vb).epsilon(1e-5f));
        CHECK(vc_out == Catch::Approx(expected_vc).epsilon(1e-5f));
    }

    SECTION("symmetrical three-phase offset")
    {
        // va=75, vb=-25, vc=-50 -> offset=(−50+75)/2=12.5
        auto [va_out, vb_out, vc_out] = svm.applyModulation(75.0f, -25.0f, -50.0f);

        float scale = 2.0f / std::sqrt(3.0f);
        float expected_va = scale * (75.0f - 12.5f);
        float expected_vb = scale * (-25.0f - 12.5f);
        float expected_vc = scale * (-50.0f - 12.5f);

        CHECK(va_out == Catch::Approx(expected_va).epsilon(1e-5f));
        CHECK(vb_out == Catch::Approx(expected_vb).epsilon(1e-5f));
        CHECK(vc_out == Catch::Approx(expected_vc).epsilon(1e-5f));
    }

    SECTION("negative voltage range")
    {
        // va=-60, vb=-100, vc=-80 -> offset=(-100-60)/2=-80
        auto [va_out, vb_out, vc_out] = svm.applyModulation(-60.0f, -100.0f, -80.0f);

        float scale = 2.0f / std::sqrt(3.0f);
        float expected_va = scale * (-60.0f - (-80.0f));
        float expected_vb = scale * (-100.0f - (-80.0f));
        float expected_vc = scale * (-80.0f - (-80.0f));

        CHECK(va_out == Catch::Approx(expected_va).epsilon(1e-5f));
        CHECK(vb_out == Catch::Approx(expected_vb).epsilon(1e-5f));
        CHECK(vc_out == Catch::Approx(expected_vc).epsilon(1e-5f));
    }

    SECTION("zero voltages")
    {
        auto [va_out, vb_out, vc_out] = svm.applyModulation(0.0f, 0.0f, 0.0f);

        CHECK(va_out == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(vb_out == Catch::Approx(0.0f).epsilon(1e-5f));
        CHECK(vc_out == Catch::Approx(0.0f).epsilon(1e-5f));
    }

    SECTION("typical three-phase sine wave snapshot")
    {
        // Typical values from 50Hz sine at specific time instant
        // va=86.6, vb=-43.3, vc=-43.3 (representing 60 degrees phase)
        auto [va_out, vb_out, vc_out] = svm.applyModulation(86.6f, -43.3f, -43.3f);

        float scale = 2.0f / std::sqrt(3.0f);
        float offset =
            (std::min({86.6f, -43.3f, -43.3f}) + std::max({86.6f, -43.3f, -43.3f})) / 2.0f;

        float expected_va = scale * (86.6f - offset);
        float expected_vb = scale * (-43.3f - offset);
        float expected_vc = scale * (-43.3f - offset);

        CHECK(va_out == Catch::Approx(expected_va).margin(1e-3f));
        CHECK(vb_out == Catch::Approx(expected_vb).margin(1e-3f));
        CHECK(vc_out == Catch::Approx(expected_vc).margin(1e-3f));
    }
}