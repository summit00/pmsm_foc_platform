#pragma once
#include <cstdint>
#include <tuple>
#include "foc.hpp"
#include "auto_setup.hpp"
#include "sensor_selector.hpp"
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
        struct Currents
        {
            float id_A = 0.0f;
            float iq_A = 0.0f;
        } currents;

        struct Voltages
        {
            float ud_V = 0.0f;
            float uq_V = 0.0f;
        } voltages;

        struct Sensors
        {
            float activeOmega_rad_Hz = 0.0f;
            float thetaOpenLoop_rad = 0.0f;
            float thetaEncoder_rad = 0.0f;
        } sensors;

        struct Reference
        {
            float targetCurrent_A = 0.0f;
            float targetSpeed_rpm = 0.0f;
            float acceleration_rpm_s = 0.0f;
            float omegaRef_rad_Hz = 0.0f;
            float polePairs = 0.0f;
        } reference;

        struct Flags
        {
            bool isClosedLoop = false;
            bool isDriveEnabled = false;
        } flags;
    };

    struct OutputRefs
    {
        float idRef_A = 0.0f;
        float iqRef_A = 0.0f;
        float injectedUd_V = 0.0f;
        float injectedUq_V = 0.0f;
        bool bypassCurrentControl = false;
        float omegaRef_rad_Hz = 0.0f;
        SensorSelector::SensorType requestedSensor = SensorSelector::SensorType::OpenLoop;
    };

    explicit ModeManager(FOC& foc, AutoSetup& autoSetup)
        : mFoc(foc), mAutoSetup(autoSetup)
    {
    }

    void setMode(ControlMode newMode)
    {
        if (mMode != newMode)
        {
            mAutoSetup.reset();
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

        if (!ctx.flags.isDriveEnabled)
        {
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
            mAutoSetup.startAutoSetup(ctx.reference.targetCurrent_A);
            mFoc.setCurrentControlGainsManual(0.5f, 0.01f);
        }

        auto refs = mAutoSetup.step(
            ctx.currents.id_A, ctx.currents.iq_A, ctx.voltages.ud_V, ctx.voltages.uq_V, ctx.sensors.thetaOpenLoop_rad, ctx.sensors.thetaEncoder_rad);

        out.idRef_A = refs.IdRef_A;
        out.iqRef_A = refs.IqRef_A;
        out.injectedUd_V = refs.UdInject_V;
        out.injectedUq_V = refs.UqInject_V;
        out.bypassCurrentControl = refs.BypassCurrentControl;
        out.omegaRef_rad_Hz = refs.OmegaRef_rad_Hz;
        out.requestedSensor = static_cast<SensorSelector::SensorType>(refs.sensorMode);

        if (refs.TriggerTuning)
        {
            mFoc.setCurrentControlGains();
        }
    }

    void runVelocityStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        out.omegaRef_rad_Hz = ctx.reference.omegaRef_rad_Hz;

        if (!ctx.flags.isClosedLoop)
        {
            out.idRef_A = ctx.reference.targetCurrent_A;
            out.iqRef_A = 0.0f;
            out.requestedSensor = SensorSelector::SensorType::OpenLoop;
        }
        else
        {
            if (++mSpeedLoopCounter >= mSpeedLoopDivider)
            {
                mSpeedLoopCounter = 0;
                std::tie(mIdRef_A_last, mIqRef_A_last) = mFoc.runSpeedControl(
                    out.omegaRef_rad_Hz, ctx.sensors.activeOmega_rad_Hz, ctx.reference.targetCurrent_A, ctx.flags.isDriveEnabled);
            }
            out.idRef_A = mIdRef_A_last;
            out.iqRef_A = mIqRef_A_last;
            out.requestedSensor = SensorSelector::SensorType::Encoder;
        }
    }

    void runTorqueStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = ctx.reference.targetCurrent_A;
        out.omegaRef_rad_Hz = ctx.sensors.activeOmega_rad_Hz;
        out.requestedSensor = SensorSelector::SensorType::Encoder;
    }

    void runPositionStep(const ExecutionContext& /*ctx*/, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = 0.0f;
        out.omegaRef_rad_Hz = 0.0f;
        out.requestedSensor = SensorSelector::SensorType::Encoder;
    }

    FOC& mFoc;
    AutoSetup& mAutoSetup;

    ControlMode mMode = ControlMode::Idle;
    uint32_t mSpeedLoopCounter = 0;
    const uint32_t mSpeedLoopDivider = 10;
    float mIdRef_A_last = 0.0f;
    float mIqRef_A_last = 0.0f;
};

} // namespace app
