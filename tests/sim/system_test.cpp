#include "motor_parameter_sets.hpp"
#include "sim_runner.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

TEST_CASE("Id step response")
{
    sim::SimRunner runner(sim::motors::motor1, 20000.0f, 10, "IdStepResponse_test.csv");

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::OPENLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 0.0f;
    ui.mAcceleration_rpm_s = 0.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(0.5f, 0.0f);

    sim::MotorState finalState = runner.getFinalState();

    CHECK(finalState.mId_A == Catch::Approx(ui.mIsAbs_mA / 1000.0f).margin(0.05f));
    CHECK(finalState.mIq_A == Catch::Approx(0.0f).margin(0.1f));
}

TEST_CASE("OpenLoop ramp", "[sil][foc]")
{
    sim::SimRunner runner(sim::motors::motor1, 20000.0f, 10, "openLoopRamp_test.csv");

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::OPENLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 500.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(1.5f, 0.0f);

    sim::MotorState finalState = runner.getFinalState();

    CHECK(finalState.mId_A == Catch::Approx(ui.mIsAbs_mA / 1000.0f).margin(0.1f));
    CHECK(finalState.mIq_A == Catch::Approx(0.0f).margin(0.1f));
    CHECK(finalState.mOmegaMech_rad_s ==
          Catch::Approx(math::rpmToRadPerSec(ui.targetSpeed_rpm)).margin(10.0f));
}