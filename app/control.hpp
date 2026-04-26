#pragma once

#include "QEI_sensor.hpp"
#include "auto_setup.hpp"
#include "dq_limiter.hpp"
#include "emk_observer.hpp"
#include "fault_manager.hpp"
#include "foc.hpp"
#include "interfaces.hpp"
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
                     UserInterface& ui)
        : mAdcSense(adc_sense), mInverter(inverter), mGateEnable(gate_enable),
          mMotorParams(motor_params), mFoc(motor_params), mUi(ui),
          mOpenLoopSensor(1.0f / 20000.0f, mOmegaRef_rad_Hz, mMotorEnabled_bool),
          mEncoderSensor(encoder,
                         1.0f / 20000.0f,
                         motor_params.polePairs,
                         2000.0f,
                         motor_params.encoderOffset_ticks),
          mEmkObserver(1.0f / 20000.0f, motor_params),
          mSensorSelector(mOpenLoopSensor, mEncoderSensor, mEmkObserver), mFaultManager(),
          mSpeedRamp(1.0f / 20000.0f)
    {
        mUdcBus_V = mAdcSense.read_bus_voltage();
        mTemp_C = mAdcSense.read_temperature_celsius();
        mFaultManager.clearFaults();
    }

    float getIdRef() const
    {
        return mIdRef_A;
    }

    float getIqRef() const
    {
        return mIqRef_A;
    }

    float getOmegaRef_rpm() const
    {
        return mOmegaRef_rad_Hz * (30.0f / std::numbers::pi_v<float>) / mMotorParams.polePairs;
    }

    uint8_t getMode() const
    {
        return static_cast<uint8_t>(mMode);
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

    float getEncoderTheta_rad() const
    {
        return mEncoderSensor.getTheta_rad();
    }

    float getEmkObserverTheta_rad() const
    {
        return mEmkObserver.getTheta_rad();
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

    void run_isr()
    {
        readUserCommands();

        if (mMotorEnabled_bool && mMode != Mode::AUTOSETUP)
        {
            mOmegaRef_rad_Hz = mSpeedRamp.update(mTargetOmega_rad_Hz, mAcceleration_rad_Hz2);
        }
        else
        {
            mSpeedRamp.reset(0.0f);
            mOmegaRef_rad_Hz = 0.0f;
        }

        PhaseCurrents currents = mAdcSense.read_amps();
        mUdcBus_V = mAdcSense.read_bus_voltage();
        mUsLimit_V = mUdcBus_V * math::INV_SQRT_3;
        mTemp_C = mAdcSense.read_temperature_celsius();

        mFaultManager.checkForFaults(currents.ia_A, currents.ic_A, mUdcBus_V, mTemp_C);
        mIsErrorrState = mFaultManager.getFaultType();

        if (mFaultManager.isFaulted())
        {
            mCmdMotorEnabled_bool = false;
            mUi.mEnable = 0;
        }

        if (mCmdMotorEnabled_bool != mMotorEnabled_bool)
        {
            mMotorEnabled_bool = mCmdMotorEnabled_bool;
            mGateEnable.set_enable(mMotorEnabled_bool);

            if (!mMotorEnabled_bool)
            {
                mFoc.resetFoc();
                mSensorSelector.updateAllSensors();
                return;
            }
        }

        std::tie(mIalpha_A, mIbeta_A) = mTransforms.clarke(currents.ia_A, currents.ic_A);
        mMotorParams.Ialpha_A = mIalpha_A;
        mMotorParams.Ibeta_A = mIbeta_A;
        mMotorParams.Ualpha_V = mUalpha_V;
        mMotorParams.Ubeta_V = mUbeta_V;

        mSensorSelector.updateAllSensors();

        float activeTheta_rad = mSensorSelector.getActiveTheta_rad();
        float activeOmega_rad_Hz = mSensorSelector.getActiveOmega_rad_Hz();

        std::tie(mId_A, mIq_A) = mTransforms.park(mIalpha_A, mIbeta_A, activeTheta_rad);

        if (mMode == Mode::OPENLOOP)
        {
            mIdRef_A = mIsAbs_A;
            mIqRef_A = 0.0f;
        }
        else if (mMode == Mode::CLOSEDLOOP)
        {
            if (++mSpeedLoopCounter_count >= mSpeedLoopDivider_count)
            {
                mSpeedLoopCounter_count = 0;
                std::tie(mIdRef_A, mIqRef_A) = mFoc.runSpeedControl(
                    mOmegaRef_rad_Hz, activeOmega_rad_Hz, -mIsAbs_A, mIsAbs_A, mMotorEnabled_bool);
            }
        }

        // Inside control.hpp -> run_isr() -> Mode::AUTOSETUP block

        else if (mMode == Mode::AUTOSETUP)
        {
            mSensorSelector.selectSensor(SensorSelector::SensorType::OpenLoop);

            if (mAutoSetup.getState() == AutoSetup::State::IDLE)
            {
                mAutoSetup.start(mIsAbs_A);
            }

            // This now handles RS and then the combined L measurement
            auto [outD, outQ, isVoltage] = mAutoSetup.run(mId_A, mIq_A, mUd_V);

            if (isVoltage) // Inductance Phase
            {
                mUd_V = outD;
                mUq_V = outQ;
                mIdRef_A = 0.0f;
                mIqRef_A = 0.0f;
            }
            else // Resistance Phase
            {
                mIdRef_A = outD;
                mIqRef_A = outQ;
                std::tie(mUd_V, mUq_V) = mFoc.runCurrentControl(
                    mIdRef_A, mIqRef_A, mId_A, mIq_A, 0.0f, -mUsLimit_V, mUsLimit_V, true);
            }

            if (mAutoSetup.isFinished())
            {
                // Re-calculate gains using the new Rs and L (Ld=Lq)
                mFoc.setCurrentControlGains();
            }
        }

        if (mMode != Mode::AUTOSETUP)
        {
            std::tie(mUd_V, mUq_V) = mFoc.runCurrentControl(mIdRef_A,
                                                            mIqRef_A,
                                                            mId_A,
                                                            mIq_A,
                                                            activeOmega_rad_Hz,
                                                            -mUsLimit_V,
                                                            mUsLimit_V,
                                                            mMotorEnabled_bool);
        }

        std::tie(mUd_V, mUq_V) = DQLimiter::applyLimit(mUd_V, mUq_V, mUsLimit_V);

        std::tie(mUalpha_V, mUbeta_V) = mTransforms.inversePark(mUd_V, mUq_V, activeTheta_rad);
        auto [Va_V, Vb_V, Vc_V] = mTransforms.inverseClarke(mUalpha_V, mUbeta_V);

        if (mMode != Mode::AUTOSETUP)
        {
            std::tie(Va_V, Vb_V, Vc_V) = spaceVectorModulation(Va_V, Vb_V, Vc_V);
        }

        mInverter.set_phase_voltages(Va_V, Vb_V, Vc_V, mUdcBus_V, mMotorEnabled_bool);

        if (++mTelemetryCounter_count >= 50)
        {
            mTelemetryCounter_count = 0;
            writeUserTelemetry();
        }
    }

  private:
    void readUserCommands()
    {
        mCmdMotorEnabled_bool = static_cast<bool>(mUi.mEnable);

        Mode newMode = static_cast<Mode>(mUi.mMode);
        if (newMode != mMode)
        {
            mMode = newMode;
            if (mMode == Mode::OPENLOOP)
            {
                mSensorSelector.selectSensor(SensorSelector::SensorType::OpenLoop);
            }
            else if (mMode == Mode::CLOSEDLOOP)
            {
                mSensorSelector.selectSensor(SensorSelector::SensorType::Encoder);
            }
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

        mUi.actualSpeed_rpm =
            (mOpenLoopSensor.getOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;
        mUi.actualSpeedEncoder_rpm =
            (mEncoderSensor.getOmega_rad_Hz() * radHzToRpm) / mMotorParams.polePairs;

        mUi.ThetaEncoder_deg = mEncoderSensor.getTheta_rad() * radToDeg;
        mUi.ThetaOpenLoop_deg = mOpenLoopSensor.getTheta_rad() * radToDeg;

        mUi.busVoltage_V = mUdcBus_V;
        mUi.Id_A = mId_A;
        mUi.Iq_A = mIq_A;
        mUi.IdRef_A = mIdRef_A;
        mUi.IqRef_A = mIqRef_A;
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
    AutoSetup mAutoSetup{mMotorParams, 1.0f / 20000.0f};

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
    Mode mMode{Mode::OPENLOOP};
    bool mMotorEnabled_bool{false};
    bool mCmdMotorEnabled_bool{false};

    // Counters
    uint8_t mTelemetryCounter_count{0};
    uint32_t mSpeedLoopCounter_count{0};
    const uint32_t mSpeedLoopDivider_count{10};
};

} // namespace app