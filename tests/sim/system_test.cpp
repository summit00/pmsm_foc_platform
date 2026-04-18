#include "motor_parameter_sets.hpp"
#include "sim_runner.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

TEST_CASE("Id step response", "[sil][foc]")
{
    // 1. Setup simulation for a heavy servo motor
    sim::SimRunner runner(sim::motors::heavy_servo, 20000.0f, 10, "spinup_test.csv");

    // 2. Configure the controller via the User Interface
    auto& ui = runner.getUi();
    ui.mMode = static_cast<uint8_t>(app::Control::Mode::OPENLOOP);
    ui.mEnable = 1;
    ui.targetSpeed_rpm = 500.0f;
    ui.mAcceleration_rpm_s = 500.0f;
    ui.mIsAbs_mA = 1500;

    // 3. Run the simulation for 0.5 seconds of simulated time
    //    (This applies a 0.1 Nm load torque to the shaft)
    runner.run(1.5f, 0.0f);

    // 4. Verify the results!
    sim::MotorState finalState = runner.getFinalState();

    // Assert that we reached the target speed within a 2.0 rad/s margin
    CHECK(finalState.mId_A == Catch::Approx(ui.mIsAbs_mA / 1000.0f).margin(0.1f));
}