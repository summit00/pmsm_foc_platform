#pragma once
#include <cstdint>
#include <tuple>
#include "foc.hpp"
#include "auto_setup.hpp"
#include "sensor_selector.hpp"
#include "math.hpp"

/**
 * @file mode_manager.hpp
 * @brief High-level motor control mode manager.
 */

namespace app
{

/**
 * @brief High-level motor control modes.
 */
enum class ControlMode : uint8_t
{
    Idle = 0,      ///< Inverter outputs are disabled, no active control.
    Autosetup,     ///< Motor commissioning and calibration routine.
    Velocity,      ///< Closed or open loop speed control mode.
    Torque,        ///< Quadrature current (torque) control mode.
    Position       ///< Closed loop rotor position control mode.
};

/**
 * @brief Manages the current control mode and execution logic of the drive.
 */
class ModeManager
{
  public:
    /**
     * @brief Execution context containing the current measurements and references.
     */
    struct ExecutionContext
    {
        /**
         * @brief Measured phase currents in the d-q reference frame.
         */
        struct Currents
        {
            float id_A = 0.0f; ///< Direct axis current in Amperes.
            float iq_A = 0.0f; ///< Quadrature axis current in Amperes.
        } currents;

        /**
         * @brief Modulating voltages in the d-q reference frame.
         */
        struct Voltages
        {
            float ud_V = 0.0f; ///< Direct axis voltage in Volts.
            float uq_V = 0.0f; ///< Quadrature axis voltage in Volts.
        } voltages;

        /**
         * @brief Sensor readings and estimated angles/speeds.
         */
        struct Sensors
        {
            float activeOmega_rad_Hz = 0.0f; ///< Selected/active rotor speed in electrical rad/s.
            float thetaOpenLoop_rad = 0.0f;  ///< Open loop rotor angle in electrical radians.
            float thetaEncoder_rad = 0.0f;   ///< Encoder rotor angle in electrical radians.
        } sensors;

        /**
         * @brief Reference inputs and configuration parameters.
         */
        struct Reference
        {
            float targetCurrent_A = 0.0f;      ///< Target current magnitude or torque reference current in Amperes.
            float targetSpeed_rpm = 0.0f;      ///< Target rotor speed in mechanical RPM.
            float acceleration_rpm_s = 0.0f;   ///< Target acceleration rate in RPM/s.
            float omegaRef_rad_Hz = 0.0f;      ///< Target rotor speed in electrical rad/s.
            float polePairs = 0.0f;            ///< Number of motor pole pairs.
        } reference;

        /**
         * @brief Control loop state flags.
         */
        struct Flags
        {
            bool isClosedLoop = false;   ///< True if running in closed loop, false otherwise.
            bool isDriveEnabled = false; ///< True if the drive power stage is enabled, false otherwise.
        } flags;
    };

    /**
     * @brief Output references computed by the mode manager.
     */
    struct OutputRefs
    {
        float idRef_A = 0.0f;              ///< Reference direct axis current in Amperes.
        float iqRef_A = 0.0f;              ///< Reference quadrature axis current in Amperes.
        float injectedUd_V = 0.0f;         ///< Injected direct axis voltage in Volts (autosetup/bypass).
        float injectedUq_V = 0.0f;         ///< Injected quadrature axis voltage in Volts (autosetup/bypass).
        bool bypassCurrentControl = false; ///< True to bypass the FOC current controllers and apply voltage directly.
        float omegaRef_rad_Hz = 0.0f;      ///< Reference rotor speed in electrical rad/s.
        SensorSelector::SensorType requestedSensor = SensorSelector::SensorType::OpenLoop; ///< Selected sensor source.
    };

    /**
     * @brief Construct a new Mode Manager object.
     * @param foc Reference to the FOC controller.
     * @param autoSetup Reference to the AutoSetup utility.
     */
    explicit ModeManager(FOC& foc, AutoSetup& autoSetup)
        : mFoc(foc), mAutoSetup(autoSetup)
    {
    }

    /**
     * @brief Transition to a new control mode.
     * @param newMode The target ControlMode.
     */
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

    /**
     * @brief Get the current control mode.
     * @return The active ControlMode.
     */
    ControlMode getMode() const { return mMode; }

    /**
     * @brief Run a single execution step of the mode manager.
     * @param ctx The current execution context (inputs/measurements).
     * @return OutputRefs The references generated for the current step.
     */
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
    /**
     * @brief Executes a step of the autosetup process.
     * @param ctx Current execution context.
     * @param out Reference to the output references structure to fill.
     */
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

    /**
     * @brief Executes a step of the velocity control mode.
     * @param ctx Current execution context.
     * @param out Reference to the output references structure to fill.
     */
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

    /**
     * @brief Executes a step of the torque control mode.
     * @param ctx Current execution context.
     * @param out Reference to the output references structure to fill.
     */
    void runTorqueStep(const ExecutionContext& ctx, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = ctx.reference.targetCurrent_A;
        out.omegaRef_rad_Hz = ctx.sensors.activeOmega_rad_Hz;
        out.requestedSensor = SensorSelector::SensorType::Encoder;
    }

    /**
     * @brief Executes a step of the position control mode.
     * @param ctx Current execution context.
     * @param out Reference to the output references structure to fill.
     */
    void runPositionStep(const ExecutionContext& /*ctx*/, OutputRefs& out)
    {
        out.idRef_A = 0.0f;
        out.iqRef_A = 0.0f;
        out.omegaRef_rad_Hz = 0.0f;
        out.requestedSensor = SensorSelector::SensorType::Encoder;
    }

    FOC& mFoc;                  ///< Reference to the field-oriented control instance.
    AutoSetup& mAutoSetup;      ///< Reference to the automatic calibration instance.

    ControlMode mMode = ControlMode::Idle; ///< The current active control mode.
    uint32_t mSpeedLoopCounter = 0;        ///< Counter for downsampling the speed controller loop.
    const uint32_t mSpeedLoopDivider = 10; ///< Downsampling divider for the speed loop.
    float mIdRef_A_last = 0.0f;            ///< The last computed d-axis current reference in Amperes.
    float mIqRef_A_last = 0.0f;            ///< The last computed q-axis current reference in Amperes.
};

} // namespace app
