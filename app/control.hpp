#pragma once

#include "QEI_sensor.hpp"
#include "auto_setup.hpp"
#include "control_drive_hardware.hpp"
#include "fault_manager.hpp"
#include "foc.hpp"
#include "interfaces.hpp"
#include "mode_manager.hpp"
#include "motor_params.hpp"
#include "open_loop_sensor.hpp"
#include "ramp_generator.hpp"
#include "sensor_selector.hpp"
#include "svm.hpp"
#include "transform.hpp"
#include "user_interface.hpp"
#include "vf_control.hpp"

#include <cstdint>
#include <numbers>
#include <tuple>

/**
 * @file control.hpp
 * @brief Core motor control loop runner and coordinator.
 */

namespace app
{

/**
 * @brief Coordinates the motor control loops, sensors, state machines, and interfaces.
 */
class Control
{
  public:
    friend class ControlDriveHardware;

    /**
     * @brief Control modes mapping to the user interface / system mode.
     */
    enum class Mode : uint8_t
    {
        OPENLOOP = 0,   ///< Open-loop speed control.
        CLOSEDLOOP = 1, ///< Closed-loop field-oriented speed control.
        AUTOSETUP = 2,  ///< Automatic tuning/calibration sequence.
    };

    /**
     * @brief Construct a new Control coordinator object.
     * @param adc_sense Reference to the ADC sensor interface.
     * @param inverter Reference to the power stage inverter interface.
     * @param gate_enable Reference to the gate driver enable interface.
     * @param encoder Reference to the encoder sensor interface.
     * @param motor_params Reference to the motor parameters container.
     * @param ui Reference to the user interface/telemetry container.
     * @param pwmPeriod_s PWM period in seconds.
     */
    explicit Control(IADC& adc_sense,
                     IInverter& inverter,
                     IEnableOutput& gate_enable,
                     IEncoder& encoder,
                     MotorParams& motor_params,
                     UserInterface& ui,
                     float pwmPeriod_s)
        : mAdcSense(adc_sense), mInverter(inverter), mGateEnable(gate_enable),
          mMotorParams(motor_params), mUi(ui),
          mOpenLoopSensor(pwmPeriod_s, mOmegaRef_rad_Hz, mMotorEnabled_bool),
          mEncoderSensor(encoder,
                         pwmPeriod_s,
                         motor_params.polePairs,
                         static_cast<float>(motor_params.encoderTicks),
                         motor_params.encoderOffset_ticks),
          mSensorSelector(mOpenLoopSensor, mEncoderSensor),
          mFoc(motor_params, pwmPeriod_s), mFaultManager(), mSpeedRamp(pwmPeriod_s),
          mAutoSetup(mMotorParams, pwmPeriod_s), mDsmHardware(*this), mDsm(mDsmHardware),
          mModeManager(mFoc, mAutoSetup),
          mVfControl(motor_params.flux_pm_Wb, motor_params.v_boost)
    {
        mUdcBus_V = mAdcSense.read_bus_voltage();
        mTemp_C = mAdcSense.read_temperature_celsius();
        mFaultManager.clearFaults();
        mDsm.initialize();
        mDsm.update();
    }

    /**
     * @brief Get the direct-axis reference current.
     * @return D-axis reference current in Amperes.
     */
    float getIdRef() const
    {
        return mIdRef_A;
    }

    /**
     * @brief Get the quadrature-axis reference current.
     * @return Q-axis reference current in Amperes.
     */
    float getIqRef() const
    {
        return mIqRef_A;
    }

    /**
     * @brief Get the reference electrical rotor speed.
     * @return Ramped reference speed in electrical rad/s.
     */
    float getOmegaRef_rad_Hz() const
    {
        return mOmegaRef_rad_Hz;
    }

    /**
     * @brief Get the active mode code for the UI.
     * @return uint8_t UI control mode code.
     */
    uint8_t getMode() const
    {
        switch (mModeManager.getMode())
        {
            case ControlMode::Velocity:
                if (mSensorSelector.getSelectedType() == SensorSelector::SensorType::OpenLoop)
                {
                    return 0; // app::Control::Mode::OPENLOOP
                }
                else
                {
                    return 1; // app::Control::Mode::CLOSEDLOOP
                }
            case ControlMode::Autosetup:
                return 2; // app::Control::Mode::AUTOSETUP
            case ControlMode::Torque:
                return 3;
            case ControlMode::Position:
                return 4;
            case ControlMode::VfControl:
                return 5;
            default:
                return 0;
        }
    }

    /**
     * @brief Get the active motor enabled state.
     * @return 1 if enabled, 0 if disabled.
     */
    uint8_t getIsEnabled() const
    {
        return static_cast<uint8_t>(mMotorEnabled_bool);
    }

    /**
     * @brief Get the current safety fault status.
     * @return Fault code from the fault manager.
     */
    uint8_t getFaultStatus() const
    {
        return mIsErrorrState;
    }

    /**
     * @brief Get the direct axis current PI controller.
     * @return Reference to the d-axis PI controller object.
     */
    const PIController& getIdController() const
    {
        return mFoc.getIdController();
    }

    /**
     * @brief Get the quadrature axis current PI controller.
     * @return Reference to the q-axis PI controller object.
     */
    const PIController& getIqController() const
    {
        return mFoc.getIqController();
    }

    /**
     * @brief Get the measured direct-axis current.
     * @return Measured d-axis current in Amperes.
     */
    float getId_A() const
    {
        return mId_A;
    }

    /**
     * @brief Get the measured quadrature-axis current.
     * @return Measured q-axis current in Amperes.
     */
    float getIq_A() const
    {
        return mIq_A;
    }

    /**
     * @brief Get the modulating direct-axis voltage command.
     * @return D-axis voltage in Volts.
     */
    float getUd_V() const
    {
        return mUd_V;
    }

    /**
     * @brief Get the modulating quadrature-axis voltage command.
     * @return Q-axis voltage in Volts.
     */
    float getUq_V() const
    {
        return mUq_V;
    }

    /**
     * @brief Get the current open-loop rotor angle.
     * @return Open-loop rotor angle in electrical radians.
     */
    float getOpenLoopTheta_rad() const
    {
        return mOpenLoopSensor.getTheta_rad();
    }

    /**
     * @brief Get the current open-loop electrical rotor speed.
     * @return Open-loop speed in electrical rad/s.
     */
    float getOpenLoopOmega_rad_Hz() const
    {
        return mOpenLoopSensor.getOmega_rad_Hz();
    }

    /**
     * @brief Get the encoder-measured rotor angle.
     * @return Encoder rotor angle in electrical radians.
     */
    float getEncoderTheta_rad() const
    {
        return mEncoderSensor.getTheta_rad();
    }

    /**
     * @brief Get the encoder-measured electrical rotor speed.
     * @return Encoder speed in electrical rad/s.
     */
    float getEncoderOmega_rad_Hz() const
    {
        return mEncoderSensor.getOmega_rad_Hz();
    }


    /**
     * @brief Get the motor stator winding resistance.
     * @return Stator resistance in Ohms.
     */
    float getRs_ohm() const
    {
        return mMotorParams.Rs_ohm;
    }

    /**
     * @brief Get the motor direct-axis stator inductance.
     * @return D-axis inductance in Henries.
     */
    float getLd_H() const
    {
        return mMotorParams.Ld_H;
    }

    /**
     * @brief Get the motor quadrature-axis stator inductance.
     * @return Q-axis inductance in Henries.
     */
    float getLq_H() const
    {
        return mMotorParams.Lq_H;
    }

    /**
     * @brief Get the motor permanent magnet flux linkage.
     * @return Flux linkage in Weber-turns.
     */
    float getPsi_pm_Wb() const
    {
        return mMotorParams.flux_pm_Wb;
    }

    /**
     * @brief Get the physical encoder offset.
     * @return Offset in encoder counts/ticks.
     */
    uint16_t getEncoderOffset_ticks() const
    {
        return mMotorParams.encoderOffset_ticks;
    }

    /**
     * @brief Get the active state of the auto-tuning sequencer.
     * @return State code of the autosetup sequence.
     */
    uint8_t getAutoSetupState() const
    {
        return static_cast<uint8_t>(mAutoSetup.getState());
    }

    /**
     * @brief Main control interrupt service routine.
     *        Executes speed ramp, reads sensors, updates state machine, runs FOC loops, and drives
     * the PWM inverter.
     */
    void run_isr()
    {
        readUserCommands();
        calculateSpeed();

        PhaseCurrents currents = readHardwareAndCheckFaults();

        handleEnableTransition();

        std::tie(mIalpha_A, mIbeta_A) = mTransforms.clarke(currents.ia_A, currents.ic_A);

        mSensorSelector.updateAllSensors();
        float activeTheta_rad = mSensorSelector.getActiveTheta_rad();
        float activeOmega_rad_Hz = mSensorSelector.getActiveOmega_rad_Hz();

        std::tie(mId_A, mIq_A) = mTransforms.park(mIalpha_A, mIbeta_A, activeTheta_rad);

        ModeManager::OutputRefs modeRefs = mModeManager.step(
            {.currents = {.id_A = mId_A, .iq_A = mIq_A},
             .voltages = {.ud_V = mUd_V, .uq_V = mUq_V},
             .sensors = {.activeOmega_rad_Hz = activeOmega_rad_Hz,
                         .thetaOpenLoop_rad = mOpenLoopSensor.getTheta_rad(),
                         .thetaEncoder_rad = mEncoderSensor.getTheta_rad()},
             .reference = {.targetCurrent_A = mIsAbs_A,
                           .targetSpeed_rpm = mUi.targetSpeed_rpm,
                           .acceleration_rpm_s = mUi.mAcceleration_rpm_s,
                           .omegaRef_rad_Hz = mOmegaRef_rad_Hz,
                           .polePairs = static_cast<float>(mMotorParams.polePairs)},
             .flags = {.isClosedLoop = (mSensorSelector.getSelectedType() !=
                                        SensorSelector::SensorType::OpenLoop),
                       .isDriveEnabled = mMotorEnabled_bool}});

        mIdRef_A = modeRefs.idRef_A;
        mIqRef_A = modeRefs.iqRef_A;
        float injectedUd_V = modeRefs.injectedUd_V;
        float injectedUq_V = modeRefs.injectedUq_V;
        bool bypassCurrentControl = modeRefs.bypassCurrentControl;
        mOmegaRef_rad_Hz = modeRefs.omegaRef_rad_Hz;

        // Apply requested sensor mode change from Mode Manager
        if (mModeManager.getMode() == ControlMode::Autosetup &&
            modeRefs.requestedSensor != mSensorSelector.getSelectedType())
        {
            mSensorSelector.selectSensor(modeRefs.requestedSensor);
        }

        float Va_V = 0.0f;
        float Vb_V = 0.0f;
        float Vc_V = 0.0f;

        if (mModeManager.getMode() == ControlMode::VfControl)
        {
            std::tie(mUd_V, mUq_V) = mVfControl.update(mOmegaRef_rad_Hz);
        }
        else
        {
            if (!bypassCurrentControl)
            {
                std::tie(mUd_V, mUq_V) = mFoc.runCurrentControl(mIdRef_A,
                                                                mIqRef_A,
                                                                mId_A,
                                                                mIq_A,
                                                                activeOmega_rad_Hz,
                                                                mUsLimit_V,
                                                                mMotorEnabled_bool);
            }
            else
            {
                mUd_V = injectedUd_V;
                mUq_V = injectedUq_V;
            }
        }

        std::tie(mUalpha_V, mUbeta_V) = mTransforms.inversePark(mUd_V, mUq_V, activeTheta_rad);
        std::tie(Va_V, Vb_V, Vc_V) = mTransforms.inverseClarke(mUalpha_V, mUbeta_V);

        auto [Va_svm_V, Vb_svm_V, Vc_svm_V] = spaceVectorModulation(Va_V, Vb_V, Vc_V);

        mInverter.set_phase_voltages(Va_V, Vb_V, Vc_V, mUdcBus_V, mMotorEnabled_bool);

        updateTelemetry();
    }

  private:
    /**
     * @brief Process drive state transitions and handle motor enable/disable logic.
     */
    void handleEnableTransition()
    {
        mDsm.update();
        mMotorEnabled_bool = mDsm.isEnabled();

        if (!mMotorEnabled_bool)
        {
            mFoc.resetFoc();
            mSensorSelector.updateAllSensors();
            mAutoSetup.reset();
            return;
        }
    }

    /**
     * @brief Read phase currents, bus voltage, and temperature from ADC, and check for faults.
     * @return PhaseCurrents Structure containing current phase measurements.
     */
    PhaseCurrents readHardwareAndCheckFaults()
    {
        PhaseCurrents currents = mAdcSense.read_amps();
        mUdcBus_V = mAdcSense.read_bus_voltage();
        mUsLimit_V = mUdcBus_V * math::INV_SQRT_3;
        mTemp_C = mAdcSense.read_temperature_celsius();

        mFaultManager.checkForFaults(currents.ia_A, currents.ic_A, mUdcBus_V, mTemp_C);
        mIsErrorrState = mFaultManager.getFaultType();

        if (mFaultManager.isFaulted())
        {
            mDsm.faultDetected();
            mCmdMotorEnabled_bool = false;
            mUi.mEnable = 0;
        }

        if (mDsm.getState() == DriveState::Fault)
        {
            mCmdMotorEnabled_bool = false;
            mUi.mEnable = 0;
        }

        return currents;
    }

    /**
     * @brief Update the speed reference based on acceleration ramps.
     */
    void calculateSpeed()
    {
        if (mMotorEnabled_bool)
        {
            if (mModeManager.getMode() == ControlMode::Velocity ||
                mModeManager.getMode() == ControlMode::VfControl)
            {
                mOmegaRef_rad_Hz = mSpeedRamp.update(mTargetOmega_rad_Hz, mAcceleration_rad_Hz2);
            }
        }
        else
        {
            mSpeedRamp.reset(0.0f);
            mOmegaRef_rad_Hz = 0.0f;
        }
    }

    /**
     * @brief Process user commands (enable/disable request, mode change) from UI.
     */
    void readUserCommands()
    {
        bool cmdEnable = static_cast<bool>(mUi.mEnable);
        if (cmdEnable && !mCmdMotorEnabled_bool)
        {
            if (mDsm.getState() == DriveState::Fault)
            {
                mDsm.resetFault();
            }
            else
            {
                mDsm.startDrive();
            }
        }
        else if (!cmdEnable && mCmdMotorEnabled_bool)
        {
            mDsm.stopDrive();
        }
        mCmdMotorEnabled_bool = cmdEnable;

        ControlMode targetMode = ControlMode::Idle;
        SensorSelector::SensorType targetSensor = SensorSelector::SensorType::Encoder;

        switch (mUi.mMode)
        {
            case 0: // OPENLOOP
                targetMode = ControlMode::Velocity;
                targetSensor = SensorSelector::SensorType::OpenLoop;
                break;
            case 1: // CLOSEDLOOP
                targetMode = ControlMode::Velocity;
                targetSensor = SensorSelector::SensorType::Encoder;
                break;
            case 2: // AUTOSETUP
                targetMode = ControlMode::Autosetup;
                targetSensor = SensorSelector::SensorType::OpenLoop;
                break;
            case 3: // TORQUE
                targetMode = ControlMode::Torque;
                targetSensor = SensorSelector::SensorType::Encoder;
                break;
            case 4: // POSITION
                targetMode = ControlMode::Position;
                targetSensor = SensorSelector::SensorType::Encoder;
                break;
            case 5: // VfControl
                targetMode = ControlMode::VfControl;
                targetSensor = SensorSelector::SensorType::OpenLoop;
                break;
            default:
                targetMode = ControlMode::Velocity;
                targetSensor = SensorSelector::SensorType::Encoder;
                break;
        }

        if (targetMode != mModeManager.getMode())
        {
            mModeManager.setMode(targetMode);
            mSpeedRamp.reset(0.0f);
        }

        if (targetSensor != mSensorSelector.getSelectedType())
        {
            mSensorSelector.selectSensor(targetSensor);
        }

        mTargetOmega_rad_Hz =
            math::mechRpmToElecRadPerSec(mUi.targetSpeed_rpm, mMotorParams.polePairs);
        mAcceleration_rad_Hz2 =
            math::mechRpmToElecRadPerSec(mUi.mAcceleration_rpm_s, mMotorParams.polePairs);
        mIsAbs_A = mUi.mIsAbs_mA / 1000.0f;
    }

    /**
     * @brief Write current control variables to UI telemetry.
     */
    void writeUserTelemetry()
    {
        constexpr float radHzToRpm = 30.0f / std::numbers::pi_v<float>;
        constexpr float radToDeg = 180.0f / std::numbers::pi_v<float>;

        mUi.demandSpeed_rpm = (mOmegaRef_rad_Hz * radHzToRpm) / mMotorParams.polePairs;
        mUi.openLoopSpeed_rpm =
            (mOpenLoopSensor.getOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;
        mUi.encoderSpeed_rpm =
            (mEncoderSensor.getOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;
        mUi.feedbackSpeed_rpm =
            (mSensorSelector.getActiveOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;
        mUi.observerSpeed_rpm = 0.0f;
        mUi.encoderAngle_deg = mEncoderSensor.getTheta_rad() * radToDeg;
        mUi.observerAngle_deg = 0.0f;
        mUi.angleError_deg = 0.0f;

        mUi.Id_A = mId_A;
        mUi.Iq_A = mIq_A;
        mUi.IdRef_A = mIdRef_A;
        mUi.IqRef_A = mIqRef_A;
        mUi.Udc_V = mUdcBus_V;
        mUi.Ud_V = mUd_V;
        mUi.Uq_V = mUq_V;
        mUi.Ualpha_V = mUalpha_V;
        mUi.Ubeta_V = mUbeta_V;
        mUi.Ialpha_A = mIalpha_A;
        mUi.Ibeta_A = mIbeta_A;
        mUi.temp_C = mTemp_C;
        mUi.errorState = static_cast<float>(mIsErrorrState);
        mUi.autoSetupState = static_cast<float>(mAutoSetup.getState());
    }

    /**
     * @brief Update telemetry log data.
     */
    void updateTelemetry()
    {
        writeUserTelemetry();
    }

    IADC& mAdcSense;            ///< ADC sensing interface.
    IInverter& mInverter;       ///< Power stage inverter interface.
    IEnableOutput& mGateEnable; ///< Gate driver enable interface.
    MotorParams& mMotorParams;  ///< Motor parameter data structure.
    UserInterface& mUi;         ///< User interface and telemetry interface.

    // Sensor Architecture
    OpenLoopSensor mOpenLoopSensor; ///< Open-loop virtual rotor position sensor.
    EncoderSensor mEncoderSensor;   ///< Encoder-based rotor position sensor.
    SensorSelector mSensorSelector; ///< Rotor position sensor selector.

    // Controllers & Math
    FOC mFoc;                   ///< Field-Oriented Control implementation.
    Transforms mTransforms;     ///< Clarke/Park mathematical transforms.
    FaultManager mFaultManager; ///< Safety fault manager.
    RampGenerator mSpeedRamp;   ///< Speed reference ramp generator.
    AutoSetup mAutoSetup;       ///< Automatic tuning sequencer.

    // Control Variables
    float mTargetOmega_rad_Hz{0.0f};   ///< Target rotor electrical speed in rad/s.
    float mOmegaRef_rad_Hz{0.0f};      ///< Ramped reference rotor electrical speed in rad/s.
    float mIdRef_A{0.0f};              ///< Reference d-axis current in Amperes.
    float mIqRef_A{0.0f};              ///< Reference q-axis current in Amperes.
    float mId_A{0.0f};                 ///< Measured d-axis current in Amperes.
    float mIq_A{0.0f};                 ///< Measured q-axis current in Amperes.
    float mIalpha_A{0.0f};             ///< Measured current in alpha coordinate in Amperes.
    float mIbeta_A{0.0f};              ///< Measured current in beta coordinate in Amperes.
    float mUd_V{0.0f};                 ///< Direct axis voltage reference in Volts.
    float mUq_V{0.0f};                 ///< Quadrature axis voltage reference in Volts.
    float mUalpha_V{0.0f};             ///< Alpha axis voltage reference in Volts.
    float mUbeta_V{0.0f};              ///< Beta axis voltage reference in Volts.
    float mIsAbs_A{0.0f};              ///< Absolute stator current limit in Amperes.
    float mAcceleration_rad_Hz2{0.0f}; ///< Acceleration rate in electrical rad/s^2.
    float mUdcBus_V{0.0f};             ///< Measured DC link bus voltage in Volts.
    float mTemp_C{0.0f};               ///< Measured power stage temperature in Celsius.
    float mUsLimit_V{};                ///< Stator voltage limit vector length in Volts.
    uint8_t mIsErrorrState{};          ///< Active fault code.

    // State Variables
    bool mMotorEnabled_bool{false};     ///< True if motor control loop is actively modulating PWM.
    bool mCmdMotorEnabled_bool{false};  ///< Requested motor enable state from host.
    AutoSetupReferences mAutoSetupRefs; ///< References for the auto-setup calibration routine.

    ControlDriveHardware mDsmHardware; ///< Drive State Machine hardware interface adapter.
    DriveStateMachine mDsm;            ///< Drive State Machine instance.
    ModeManager mModeManager;          ///< Control mode manager instance.
    VfControl mVfControl;              ///< VfControl instance.

    // Counters
    uint8_t mTelemetryCounter_count{0};         ///< Telemetry loop divider count.
    uint32_t mSpeedLoopCounter_count{0};        ///< Speed loop downsampling count.
    const uint32_t mSpeedLoopDivider_count{10}; ///< Speed loop downsampling divider.
};

// Inline definitions for ControlDriveHardware methods

/**
 * @brief Construct a new ControlDriveHardware adapter.
 * @param parent Reference to the parent Control class instance.
 */
inline ControlDriveHardware::ControlDriveHardware(Control& parent) : mParent(parent)
{
}

/**
 * @brief Implementation to enable the physical gate driver.
 */
inline void ControlDriveHardware::enableGateDriver()
{
    mParent.mGateEnable.set_enable(true);
}

/**
 * @brief Implementation to disable the physical gate driver.
 */
inline void ControlDriveHardware::disableGateDriver()
{
    mParent.mGateEnable.set_enable(false);
}

/**
 * @brief Implementation to enable PWM outputs (unused on this platform).
 */
inline void ControlDriveHardware::enablePwm()
{
}

/**
 * @brief Implementation to disable PWM outputs (unused on this platform).
 */
inline void ControlDriveHardware::disablePwm()
{
}

/**
 * @brief Implementation to reset internal control states.
 */
inline void ControlDriveHardware::resetControllers()
{
    mParent.mFoc.resetFoc();
    mParent.mAutoSetup.reset();
}

/**
 * @brief Implementation to query if the hardware is free of faults.
 * @return true if hardware is ready, false if faulted.
 */
inline bool ControlDriveHardware::isHardwareReady() const
{
    return !mParent.mFaultManager.isFaulted();
}

/**
 * @brief Implementation to query if the gate driver is ready (always true on this hardware).
 * @return true.
 */
inline bool ControlDriveHardware::isGateDriverReady() const
{
    return true;
}

/**
 * @brief Implementation to query if there is any active hardware fault.
 * @return true if faulted, false otherwise.
 */
inline bool ControlDriveHardware::hasActiveFault() const
{
    return mParent.mFaultManager.isFaulted();
}

} // namespace app
