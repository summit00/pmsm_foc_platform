#pragma once

#include "QEI_sensor.hpp"
#include "auto_setup.hpp"
#include "control_drive_hardware.hpp"
#include "emk_observer.hpp"
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

#include <cstdint>
#include <numbers>
#include <tuple>

namespace app
{

class Control
{
  public:
    friend class ControlDriveHardware;
    enum class Mode : uint8_t
    {
        OPENLOOP = 0,
        CLOSEDLOOP = 1,
        AUTOSETUP = 2,
    };

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
          mEmkObserver(pwmPeriod_s, motor_params),
          mSensorSelector(mOpenLoopSensor, mEncoderSensor, mEmkObserver),
          mFoc(motor_params, pwmPeriod_s), mFaultManager(), mSpeedRamp(pwmPeriod_s),
          mAutoSetup(mMotorParams, pwmPeriod_s), mDsmHardware(*this), mDsm(mDsmHardware),
          mModeManager(mFoc, mAutoSetup, mSpeedRamp)
    {
        mUdcBus_V = mAdcSense.read_bus_voltage();
        mTemp_C = mAdcSense.read_temperature_celsius();
        mFaultManager.clearFaults();
        mDsm.initialize();
        mDsm.update();
    }

    float getIdRef() const
    {
        return mIdRef_A;
    }

    float getIqRef() const
    {
        return mIqRef_A;
    }

    float getOmegaRef_rad_Hz() const
    {
        return mOmegaRef_rad_Hz;
    }

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
            default:
                return 0;
        }
    }

    uint8_t getIsEnabled() const
    {
        return static_cast<uint8_t>(mMotorEnabled_bool);
    }

    uint8_t getFaultStatus() const
    {
        return mIsErrorrState;
    }

    const PIController& getIdController() const
    {
        return mFoc.getIdController();
    }

    const PIController& getIqController() const
    {
        return mFoc.getIqController();
    }

    float getId_A() const
    {
        return mId_A;
    }

    float getIq_A() const
    {
        return mIq_A;
    }

    float getUd_V() const
    {
        return mUd_V;
    }

    float getUq_V() const
    {
        return mUq_V;
    }

    float getOpenLoopTheta_rad() const
    {
        return mOpenLoopSensor.getTheta_rad();
    }

    float getOpenLoopOmega_rad_Hz() const
    {
        return mOpenLoopSensor.getOmega_rad_Hz();
    }

    float getEncoderTheta_rad() const
    {
        return mEncoderSensor.getTheta_rad();
    }

    float getEncoderOmega_rad_Hz() const
    {
        return mEncoderSensor.getOmega_rad_Hz();
    }

    float getEmkObserverTheta_rad() const
    {
        return mEmkObserver.getTheta_rad();
    }

    float getEmkObserverOmega_rad_Hz() const
    {
        return mEmkObserver.getOmega_rad_Hz();
    }

    float getRs_ohm() const
    {
        return mMotorParams.Rs_ohm;
    }

    float getLd_H() const
    {
        return mMotorParams.Ld_H;
    }

    float getLq_H() const
    {
        return mMotorParams.Lq_H;
    }

    float getPsi_pm_Wb() const
    {
        return mMotorParams.flux_pm_Wb;
    }

    uint16_t getEncoderOffset_ticks() const
    {
        return mMotorParams.encoderOffset_ticks;
    }

    uint8_t getAutoSetupState() const
    {
        return static_cast<uint8_t>(mAutoSetup.getState());
    }

    void run_isr()
    {
        readUserCommands();
        calculateSpeed();

        PhaseCurrents currents = readHardwareAndCheckFaults();

        handleEnableTransition();

        std::tie(mIalpha_A, mIbeta_A) = mTransforms.clarke(currents.ia_A, currents.ic_A);
        setSensorlessData();

        mSensorSelector.updateAllSensors();
        float activeTheta_rad = mSensorSelector.getActiveTheta_rad();
        float activeOmega_rad_Hz = mSensorSelector.getActiveOmega_rad_Hz();

        std::tie(mId_A, mIq_A) = mTransforms.park(mIalpha_A, mIbeta_A, activeTheta_rad);

        ModeManager::OutputRefs modeRefs =
            mModeManager.step({.activeOmega_rad_Hz = activeOmega_rad_Hz,
                               .thetaOpenLoop_rad = mOpenLoopSensor.getTheta_rad(),
                               .thetaEncoder_rad = mEncoderSensor.getTheta_rad(),
                               .id_A = mId_A,
                               .iq_A = mIq_A,
                               .ud_V = mUd_V,
                               .uq_V = mUq_V,
                               .targetCurrent_A = mIsAbs_A,
                               .targetSpeed_rpm = mUi.targetSpeed_rpm,
                               .acceleration_rpm_s = mUi.mAcceleration_rpm_s,
                               .polePairs = static_cast<float>(mMotorParams.polePairs),
                               .omegaRef_rad_Hz = mOmegaRef_rad_Hz,
                               .isClosedLoop = (mSensorSelector.getSelectedType() !=
                                                SensorSelector::SensorType::OpenLoop),
                               .isDriveEnabled = mMotorEnabled_bool});

        mIdRef_A = modeRefs.idRef_A;
        mIqRef_A = modeRefs.iqRef_A;
        float injectedUd_V = modeRefs.injectedUd_V;
        float injectedUq_V = modeRefs.injectedUq_V;
        bool bypassCurrentControl = modeRefs.bypassCurrentControl;
        mOmegaRef_rad_Hz = modeRefs.omegaRef_rad_Hz;

        // Apply requested sensor mode change from Mode Manager
        if (mModeManager.getMode() == ControlMode::Autosetup &&
            modeRefs.requestedSensorMode != static_cast<uint8_t>(mSensorSelector.getSelectedType()))
        {
            mSensorSelector.selectSensor(
                static_cast<SensorSelector::SensorType>(modeRefs.requestedSensorMode));
        }

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

        std::tie(mUalpha_V, mUbeta_V) = mTransforms.inversePark(mUd_V, mUq_V, activeTheta_rad);
        auto [Va_V, Vb_V, Vc_V] = mTransforms.inverseClarke(mUalpha_V, mUbeta_V);

        mInverter.set_phase_voltages(Va_V, Vb_V, Vc_V, mUdcBus_V, mMotorEnabled_bool);

        updateTelemetry();
    }

  private:
    void setSensorlessData()
    {
        mMotorParams.Ialpha_A = mIalpha_A;
        mMotorParams.Ibeta_A = mIbeta_A;
        mMotorParams.Ualpha_V = mUalpha_V;
        mMotorParams.Ubeta_V = mUbeta_V;
    }

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

    void calculateSpeed()
    {
        if (mMotorEnabled_bool)
        {
            if (mModeManager.getMode() == ControlMode::Velocity)
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
            default:
                targetMode = ControlMode::Velocity;
                targetSensor = SensorSelector::SensorType::Encoder;
                break;
        }

        if (targetMode != mModeManager.getMode())
        {
            mModeManager.setMode(targetMode);
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
        mUi.observerSpeed_rpm =
            (mEmkObserver.getOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;
        mUi.encoderAngle_deg = mEncoderSensor.getTheta_rad() * radToDeg;
        mUi.observerAngle_deg = mEmkObserver.getTheta_rad() * radToDeg;
        mUi.angleError_deg =
            math::compute_angle_error(mEncoderSensor.getTheta_rad(), mEmkObserver.getTheta_rad()) *
            radToDeg;

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

    void updateTelemetry()
    {
        writeUserTelemetry();
    }

    IADC& mAdcSense;
    IInverter& mInverter;
    IEnableOutput& mGateEnable;
    MotorParams& mMotorParams;
    UserInterface& mUi;

    // Sensor Architecture
    OpenLoopSensor mOpenLoopSensor;
    EncoderSensor mEncoderSensor;
    EmkObserver mEmkObserver;
    SensorSelector mSensorSelector;

    // Controllers & Math
    FOC mFoc;
    Transforms mTransforms;
    FaultManager mFaultManager;
    RampGenerator mSpeedRamp;
    AutoSetup mAutoSetup;

    // Control Variables
    float mTargetOmega_rad_Hz{0.0f};
    float mOmegaRef_rad_Hz{0.0f};
    float mIdRef_A{0.0f};
    float mIqRef_A{0.0f};
    float mId_A{0.0f};
    float mIq_A{0.0f};
    float mIalpha_A{0.0f};
    float mIbeta_A{0.0f};
    float mUd_V{0.0f};
    float mUq_V{0.0f};
    float mUalpha_V{0.0f};
    float mUbeta_V{0.0f};
    float mIsAbs_A{0.0f};
    float mAcceleration_rad_Hz2{0.0f};
    float mUdcBus_V{0.0f};
    float mTemp_C{0.0f};
    float mUsLimit_V{};
    uint8_t mIsErrorrState{};
    // State Variables
    bool mMotorEnabled_bool{false};
    bool mCmdMotorEnabled_bool{false};
    AutoSetupReferences mAutoSetupRefs;

    ControlDriveHardware mDsmHardware;
    DriveStateMachine mDsm;
    ModeManager mModeManager;

    // Counters
    uint8_t mTelemetryCounter_count{0};
    uint32_t mSpeedLoopCounter_count{0};
    const uint32_t mSpeedLoopDivider_count{10};
};

// Inline definitions for ControlDriveHardware methods
inline ControlDriveHardware::ControlDriveHardware(Control& parent) : mParent(parent)
{
}

inline void ControlDriveHardware::enableGateDriver()
{
    mParent.mGateEnable.set_enable(true);
}

inline void ControlDriveHardware::disableGateDriver()
{
    mParent.mGateEnable.set_enable(false);
}

inline void ControlDriveHardware::enablePwm()
{
}
inline void ControlDriveHardware::disablePwm()
{
}

inline void ControlDriveHardware::resetControllers()
{
    mParent.mFoc.resetFoc();
    mParent.mAutoSetup.reset();
}

inline bool ControlDriveHardware::isHardwareReady() const
{
    return !mParent.mFaultManager.isFaulted();
}

inline bool ControlDriveHardware::isGateDriverReady() const
{
    return true;
}

inline bool ControlDriveHardware::hasActiveFault() const
{
    return mParent.mFaultManager.isFaulted();
}

} // namespace app
