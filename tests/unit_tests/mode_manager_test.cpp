#include "mode_manager.hpp"
#include "motor_params.hpp"
#include <catch2/catch_test_macros.hpp>

using namespace app;

TEST_CASE("ModeManager transitions and stepping")
{
    float pwmPeriod_s = 0.0001f;
    MotorParams motor_params;
    motor_params.polePairs = 4;
    motor_params.encoderTicks = 4000;
    motor_params.encoderOffset_ticks = 0;

    FOC foc(motor_params, pwmPeriod_s);
    AutoSetup autoSetup(motor_params, pwmPeriod_s);
    RampGenerator speedRamp(pwmPeriod_s);

    ModeManager mm(foc, autoSetup, speedRamp);

    SECTION("Default mode is Idle")
    {
        REQUIRE(mm.getMode() == ControlMode::Idle);
    }

    SECTION("setMode transitions correctly")
    {
        mm.setMode(ControlMode::Velocity);
        REQUIRE(mm.getMode() == ControlMode::Velocity);

        mm.setMode(ControlMode::Autosetup);
        REQUIRE(mm.getMode() == ControlMode::Autosetup);
    }

    SECTION("step on disabled drive returns zero output and resets internal states")
    {
        mm.setMode(ControlMode::Velocity);
        
        ModeManager::ExecutionContext ctx;
        ctx.isDriveEnabled = false;
        ctx.targetCurrent_A = 1.5f;

        auto out = mm.step(ctx);
        REQUIRE(out.idRef_A == 0.0f);
        REQUIRE(out.iqRef_A == 0.0f);
        REQUIRE(out.omegaRef_rad_Hz == 0.0f);
    }

    SECTION("Velocity mode open loop step")
    {
        mm.setMode(ControlMode::Velocity);

        ModeManager::ExecutionContext ctx;
        ctx.isDriveEnabled = true;
        ctx.omegaRef_rad_Hz = 25.0f;
        ctx.targetCurrent_A = 1.5f;        //open loop active current
        ctx.isClosedLoop = false;          //open loop speed
        ctx.polePairs = 4.0f;

        auto out = mm.step(ctx);
        // In open-loop velocity control, idRef should equal the target current and iqRef should be 0
        REQUIRE(out.idRef_A == 1.5f);
        REQUIRE(out.iqRef_A == 0.0f);
        REQUIRE(out.omegaRef_rad_Hz == 25.0f);
        REQUIRE(out.requestedSensorMode == 0); // OpenLoop
    }

    SECTION("Torque mode step")
    {
        mm.setMode(ControlMode::Torque);

        ModeManager::ExecutionContext ctx;
        ctx.isDriveEnabled = true;
        ctx.targetCurrent_A = 2.0f;
        ctx.activeOmega_rad_Hz = 15.0f;
        ctx.polePairs = 4.0f;

        auto out = mm.step(ctx);
        REQUIRE(out.idRef_A == 0.0f);
        REQUIRE(out.iqRef_A == 2.0f);
        REQUIRE(out.omegaRef_rad_Hz == 15.0f);
        REQUIRE(out.requestedSensorMode == 1); // Encoder
    }
}
