#pragma once
#include <cstdint>
#include <tuple>
#include "foc.hpp"
#include "auto_setup.hpp"
#include "ramp_generator.hpp"
#include "math.hpp"

namespace app
{

/**
 * @brief High-level motor control modes.
 */
enum class ControlMode : uint8_t
{
    Idle = 0,
    Autosetup,
    Velocity,
    Torque,
    Position
};

class ModeManager
{
  public:
    struct ExecutionContext
    {
        float activeOmega_rad_Hz;
        float thetaOpenLoop_rad;
        float thetaEncoder_rad;
        float id_A;
        float iq_A;
        float ud_V;
        float uq_V;
        float targetCurrent_A;
        float targetSpeed_rpm;
        float acceleration_rpm_s;
        float polePairs;
        float omegaRef_rad_Hz;
        bool isClosedLoop;
        bool isDriveEnabled;
    };

    struct OutputRefs
    {
        float idRef_A = 0.0f;
        float iqRef_A = 0.0f;
        float injectedUd_V = 0.0f;
        float injectedUq_V = 0.0f;
        bool bypassCurrentControl = false;
        float omegaRef_rad_Hz = 0.0f;
        uint8_t requestedSensorMode = 0; // 0 = OpenLoop, 1 = Encoder, 2 = EmkObserver
    };

    explicit ModeManager(FOC& foc, AutoSetup& autoSetup, RampGenerator& speedRamp)
        : mFoc(foc), mAutoSetup(autoSetup), mSpeedRamp(speedRamp)
    {
    }

    void setMode(ControlMode newMode)
    {
        if (mMode != newMode)
        {
            mAutoSetup.reset();
            mSpeedRamp.reset(0.0f);
            mSpeedLoopCounter = 0;
            mIdRef_A_last = 0.0f;
            mIqRef_A_last = 0.0f;
            mMode = newMode;
        }
    }

    ControlMode getMode() const { return mMode; }

    OutputRefs step(const ExecutionContext& ctx)
    {
        OutputRefs out;

        if (!ctx.isDriveEnabled)
        {
            mSpeedRamp.reset(0.0f);
            mAutoSetup.reset();
            return out;
        }

        switch (mMode)
        {
            case ControlMode::Idle:
                break;

            case ControlMode::Autosetup:
                runAutosetupStep(ctx, out);
                break;

            case ControlMode::Velocity:
                runVelocityStep(ctx, out);
                break;

            case ControlMode::Torque:
                runTorqueStep(ctx, out);
                break;

            case ControlMode::Position:
                runPositionStep(ctx, out);
                break;
        }

        return out;
    }

  private:
    void runAutosetupStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        if (mAutoSetup.getState() == AutoSetup::State::IDLE)
        {
            mAutoSetup.startAutoSetup(ctx.targetCurrent_A);
            mFoc.setCurrentControlGainsManual(0.5f, 0.01f);
        }

        auto refs = mAutoSetup.step(
            ctx.id_A, ctx.iq_A, ctx.ud_V, ctx.uq_V, ctx.thetaOpenLoop_rad, ctx.thetaEncoder_rad);

        out.idRef_A = refs.IdRef_A;
        out.iqRef_A = refs.IqRef_A;
        out.injectedUd_V = refs.UdInject_V;
        out.injectedUq_V = refs.UqInject_V;
        out.bypassCurrentControl = refs.BypassCurrentControl;
        out.omegaRef_rad_Hz = refs.OmegaRef_rad_Hz;
        out.requestedSensorMode = refs.sensorMode;

        if (refs.TriggerTuning)
        {
            mFoc.setCurrentControlGains();
        }
    }

    void runVelocityStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        out.omegaRef_rad_Hz = ctx.omegaRef_rad_Hz;

        if (!ctx.isClosedLoop)
        {
            out.idRef_A = ctx.targetCurrent_A;
            out.iqRef_A = 0.0f;
            out.requestedSensorMode = 0; // OpenLoop
        }
        else
        {
            if (++mSpeedLoopCounter >= mSpeedLoopDivider)
            {
                mSpeedLoopCounter = 0;
                std::tie(mIdRef_A_last, mIqRef_A_last) = mFoc.runSpeedControl(
                    out.omegaRef_rad_Hz, ctx.activeOmega_rad_Hz, ctx.targetCurrent_A, ctx.isDriveEnabled);
            }
            out.idRef_A = mIdRef_A_last;
            out.iqRef_A = mIqRef_A_last;
            out.requestedSensorMode = 1; // Encoder
        }
    }

    void runTorqueStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = ctx.targetCurrent_A;
        out.omegaRef_rad_Hz = ctx.activeOmega_rad_Hz;
        out.requestedSensorMode = 1; // Encoder
    }

    void runPositionStep(const ExecutionContext& /*ctx*/, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = 0.0f;
        out.omegaRef_rad_Hz = 0.0f;
        out.requestedSensorMode = 1; // Encoder
    }

    FOC& mFoc;
    AutoSetup& mAutoSetup;
    RampGenerator& mSpeedRamp;

    ControlMode mMode = ControlMode::Idle;
    uint32_t mSpeedLoopCounter = 0;
    const uint32_t mSpeedLoopDivider = 10;
    float mIdRef_A_last = 0.0f;
    float mIqRef_A_last = 0.0f;
};

} // namespace app
