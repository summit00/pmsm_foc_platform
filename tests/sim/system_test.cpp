#include "motor_parameter_sets.hpp"
#include "sim_runner.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

TEST_CASE("Id step response")
{
    sim::SimpleLoad load;
    sim::SimRunner runner(sim::motors::motor1, load, "IdStepResponse_test.csv");

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

TEST_CASE("OpenLoop ramp")
{
    sim::SimpleLoad load;
    sim::SimRunner runner(sim::motors::motor1, load, "openLoopRamp_test.csv");

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::OPENLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 2000.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(1.5f, 0.0f);

    sim::MotorState finalState = runner.getFinalState();

    CHECK(finalState.mId_A == Catch::Approx(ui.mIsAbs_mA / 1000.0f).margin(0.1f));
    CHECK(finalState.mIq_A == Catch::Approx(0.0f).margin(0.1f));
    CHECK(finalState.mOmegaMech_rad_s ==
          Catch::Approx(math::rpmToRadPerSec(ui.targetSpeed_rpm)).margin(10.0f));
}

TEST_CASE("ClosedLoop step response")
{

    sim::SimpleLoad load;
    sim::SimRunner runner(sim::motors::motor1, load, "ClosedLoopStep_test.csv");

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::CLOSEDLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 5000.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(0.3f, 0.0f);

    sim::MotorState finalState = runner.getFinalState();

    CHECK(finalState.mId_A == Catch::Approx(0.0f).margin(0.1f));
    CHECK(finalState.mOmegaMech_rad_s ==
          Catch::Approx(math::rpmToRadPerSec(ui.targetSpeed_rpm)).margin(10.0f));
}

TEST_CASE("External Load Response")
{
    sim::SimpleLoad load;
    load.setBrakingTorque(0.001f);

    sim::SimRunner runner(sim::motors::motor1, load, "LoadTest.csv");

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::CLOSEDLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 500.0f;
    ui.mIsAbs_mA = 3000;

    runner.run(1.0f);

    load.setBrakingTorque(0.01f);
    runner.run(1.0f);
}

TEST_CASE("Obserever Test")
{

    sim::SimpleLoad load;
    sim::SimRunner runner(sim::motors::motor1, load, "Observer_test.csv");
    auto& control = runner.getControl();

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::CLOSEDLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 5000.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(0.3f, 0.0f);
    float observerError_deg = math::compute_angle_error(control.getEmkObserverTheta_rad(),
                                                        control.getEncoderTheta_rad()) *
                              (180.0f / std::numbers::pi_v<float>);
    CHECK(observerError_deg == Catch::Approx(0.0f).margin(5.0f));

    sim::MotorState finalState = runner.getFinalState();

    CHECK(finalState.mId_A == Catch::Approx(0.0f).margin(0.1f));
    CHECK(finalState.mOmegaMech_rad_s ==
          Catch::Approx(math::rpmToRadPerSec(ui.targetSpeed_rpm)).margin(10.0f));

    ui.targetSpeed_rpm = 700.0f;
    runner.run(0.3f, 0.0f);
    observerError_deg = math::compute_angle_error(control.getEmkObserverTheta_rad(),
                                                  control.getEncoderTheta_rad()) *
                        (180.0f / std::numbers::pi_v<float>);
    CHECK(observerError_deg == Catch::Approx(0.0f).margin(5.0f));
}

TEST_CASE("Auto-Setup Test", "[debug]")
{
    sim::SimpleLoad load;
    sim::SimRunner runner(sim::motors::motor1, load, "AutoSetup_test.csv");
    auto& control = runner.getControl();

    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::AUTOSETUP);
    float oldResistance_ohm = control.getRs_ohm();
    float oldLd_H = control.getLd_H();
    float oldLq_H = control.getLq_H();
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 0.0f;
    ui.mAcceleration_rpm_s = 0.0f;
    ui.mIsAbs_mA = 1500;

    runner.run(2.0f, 0.0f);

    CHECK(oldResistance_ohm != control.getRs_ohm());
    CHECK(oldLd_H != control.getLd_H());
    CHECK(oldLq_H != control.getLq_H());
    CHECK(control.getRs_ohm() == Catch::Approx(sim::motors::motor1.Rs_ohm).margin(0.02f));
    CHECK(control.getLd_H() == Catch::Approx(sim::motors::motor1.Ld_H).margin(0.00002f));
    CHECK(control.getLq_H() == Catch::Approx(sim::motors::motor1.Lq_H).margin(0.00002f));
}