#pragma once
#include "math.hpp"
#include "motor_params.hpp"
#include "powerstage_config.hpp"
#include "ramp_generator.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <tuple>

namespace app
{

struct AutoSetupReferences
{
    float IdRef_A{0.0f};
    float IqRef_A{0.0f};
    float UdInject_V{0.0f};
    float UqInject_V{0.0f};
    bool BypassCurrentControl{false};
    bool TriggerTuning{false};
    float OmegaRef_rad_Hz{0.0f};
    uint8_t sensorMode{0};
};

class AutoSetup
{
  public:
    enum class State : uint8_t
    {
        IDLE,
        RS_RAMP_UP,
        RS_MEASURE,
        RS_RAMP_DOWN,
        LS_MEASURE,
        LS_RAMP_DOWN,
        PI_TUNE,
        ALIGN_FWD,
        ALIGN_BWD,
        SPEED_RAMP_DOWN,
        PSI_MEASUREMENT,
        SPEED_RAMP_STOP,
        FINISHED
    };

    struct Config
    {
        static constexpr uint32_t RS_SETTLE_TICKS = 15000;
        static constexpr uint32_t RS_MEASURE_SAMPLES = 10000;
        static constexpr uint32_t LS_MEASURE_SAMPLES = 10000;
        static constexpr uint32_t PSI_MEASURE_SAMPLES = 20000;
        static constexpr uint32_t ALIGN_SETTLE_SAMPLES = 20000;

        static constexpr float RS_RAMP_RATE_A_S = 2.0f;
        static constexpr float RS_DISCHARGE_RATE_A_S = 5.0f;
        static constexpr float ALIGN_ACCEL_RAD_S2 = 150.0f;
        static constexpr float STOP_ACCEL_RAD_S2 = 1000.0f;

        static constexpr float LS_INJECTION_FREQ_HZ = 1000.0f;
        static constexpr float ALIGN_SPEED_RAD_S = 300.0f;
        static constexpr float SETTLE_THRESHOLD_RAD_S = 0.1f;
    };

    explicit AutoSetup(MotorParams& params, float pwmPeriod_s)
        : mParams(params), mRamp(pwmPeriod_s), mRampSpeed(pwmPeriod_s), mPwmPeriod_s(pwmPeriod_s)
    {
    }

    AutoSetupReferences step(float Id_A,
                             float Iq_A,
                             float UdApplied_V,
                             float UqApplied_V,
                             float thetaOpenLoop_rad,
                             float thetaEncoder_rad)
    {
        AutoSetupReferences ref;

        switch (mState)
        {
            case State::IDLE:
                break;

            case State::RS_RAMP_UP:
                if (rampCurrent(mTargetCurrent_A, Config::RS_RAMP_RATE_A_S, ref.IdRef_A))
                    mState = State::RS_MEASURE;
                break;

            case State::RS_MEASURE:
                ref.IdRef_A = mTargetCurrent_A;
                if (measureRs(Id_A, Iq_A, UdApplied_V))
                    mState = State::RS_RAMP_DOWN;
                break;

            case State::RS_RAMP_DOWN:
                if (rampCurrent(0.0f, Config::RS_DISCHARGE_RATE_A_S, ref.IdRef_A))
                    mState = State::LS_MEASURE;
                break;

            case State::LS_MEASURE:
                ref.BypassCurrentControl = true;
                if (measureLs(Id_A, ref.UdInject_V))
                    mState = State::LS_RAMP_DOWN;
                break;

            case State::LS_RAMP_DOWN:
                ref.BypassCurrentControl = true;
                if (rampVoltageDown(ref.UdInject_V))
                    mState = State::PI_TUNE;
                break;

            case State::PI_TUNE:
                ref.TriggerTuning = true;
                resetPhase();
                mState = State::ALIGN_FWD;
                break;

            case State::ALIGN_FWD:
                ref.IdRef_A = mTargetCurrent_A;
                ref.sensorMode = 0;
                if (rampSpeed(
                        Config::ALIGN_SPEED_RAD_S, Config::ALIGN_ACCEL_RAD_S2, ref.OmegaRef_rad_Hz))
                {
                    if (averageOffset(thetaOpenLoop_rad, thetaEncoder_rad, mOffsetFwd_rad))
                    {
                        resetPhase();
                        mState = State::ALIGN_BWD;
                    }
                }
                break;

            case State::ALIGN_BWD:
                ref.IdRef_A = mTargetCurrent_A;
                ref.sensorMode = 0;
                ref.OmegaRef_rad_Hz =
                    mRampSpeed.update(-Config::ALIGN_SPEED_RAD_S, Config::ALIGN_ACCEL_RAD_S2);
                if (std::abs(ref.OmegaRef_rad_Hz + Config::ALIGN_SPEED_RAD_S) <
                    Config::SETTLE_THRESHOLD_RAD_S)
                {
                    if (averageOffset(thetaOpenLoop_rad, thetaEncoder_rad, mOffsetBwd_rad))
                    {
                        calculateFinalOffset();
                        resetPhase();
                        mState = State::SPEED_RAMP_DOWN;
                    }
                }
                break;

            case State::SPEED_RAMP_DOWN:
                ref.IdRef_A = mTargetCurrent_A;

                if (rampSpeed(0.0f, Config::STOP_ACCEL_RAD_S2, ref.OmegaRef_rad_Hz))
                {
                    resetPhase();
                    mState = State::PSI_MEASUREMENT;
                }
                break;

            case State::PSI_MEASUREMENT:
                ref.IdRef_A = mTargetCurrent_A;
                ref.OmegaRef_rad_Hz =
                    mRampSpeed.update(-Config::ALIGN_SPEED_RAD_S, Config::ALIGN_ACCEL_RAD_S2);
                if (std::abs(ref.OmegaRef_rad_Hz + Config::ALIGN_SPEED_RAD_S) <
                    Config::SETTLE_THRESHOLD_RAD_S)
                {
                    if (measurePsi(Id_A, Iq_A, UqApplied_V, ref.OmegaRef_rad_Hz))
                    {
                        resetPhase();
                        mState = State::SPEED_RAMP_STOP;
                    }
                }
                break;

            case State::SPEED_RAMP_STOP:
                ref.IdRef_A = mTargetCurrent_A;

                if (rampSpeed(0.0f, Config::STOP_ACCEL_RAD_S2, ref.OmegaRef_rad_Hz))
                {
                    resetPhase();
                    mState = State::FINISHED;
                }
                break;

            case State::FINISHED:
                resetPhase();
                break;

            default:
                break;
        }

        return ref;
    }

    void startAutoSetup(float IsAbs_A)
    {
        mTargetCurrent_A = IsAbs_A;
        mInjectionVoltage_V = 0.0f;
        mTime_s = 0.0f;
        mState = State::RS_RAMP_UP;
        resetPhase();
    }

    State getState() const
    {
        return mState;
    }
    bool isFinished() const
    {
        return mState == State::FINISHED;
    }
    void reset()
    {
        mState = State::IDLE;
    }

  private:
    bool rampCurrent(float target_A, float rampRate_A_Hz, float& id_ref_out_A)
    {
        id_ref_out_A = mRamp.update(target_A, rampRate_A_Hz);
        return std::abs(id_ref_out_A - target_A) < 0.01f;
    }

    bool rampSpeed(float omegaRef_rad_Hz, float accel_rad_s2, float& omegaOut_rad_s)
    {
        omegaOut_rad_s = mRampSpeed.update(omegaRef_rad_Hz, accel_rad_s2);
        return std::abs(omegaOut_rad_s - omegaRef_rad_Hz) < Config::SETTLE_THRESHOLD_RAD_S;
    }

    bool measureRs(float Id_A, float Iq_A, float Ud_V)
    {
        if (++mTimer < Config::RS_SETTLE_TICKS)
            return false;

        if (std::abs(Iq_A) > (mTargetCurrent_A * 0.05f))
        {
            resetPhase();
            return false;
        }

        mUdSum += Ud_V;
        mIdSum += Id_A;

        if (++mSamples >= Config::RS_MEASURE_SAMPLES)
        {
            float meanI = mIdSum / mSamples;
            float meanU = mUdSum / mSamples;

            if (meanI < (mTargetCurrent_A * 0.90f))
            {
                resetPhase();
                return false;
            }

            mParams.RTotal_ohm = meanU / meanI;
            auto config = getPowerStageConfig();
            mParams.Rs_ohm = mParams.RTotal_ohm - config.RtotalOffset_ohm;
            resetPhase();
            return true;
        }
        return false;
    }

    bool measureLs(float Id_A, float& UdInject_V)
    {
        constexpr float omega_rad_Hz = 2.0f * math::PI * Config::LS_INJECTION_FREQ_HZ;

        if (mInjectionVoltage_V == 0.0f)
        {
            mInjectionVoltage_V = mParams.RTotal_ohm * mTargetCurrent_A;
        }

        mTime_s += mPwmPeriod_s;
        UdInject_V = mInjectionVoltage_V * math::sin(omega_rad_Hz * mTime_s);
        mMeasuredCurrentPeak_A = std::max(mMeasuredCurrentPeak_A, std::abs(Id_A));

        if (mSamples == 0)
        {
            handleVoltageSearch();
            return false;
        }

        if (++mSamples > Config::LS_MEASURE_SAMPLES)
        {
            const float reactance_ohm =
                std::sqrt(std::max(0.0f,
                                   math::square(mInjectionVoltage_V / mMeasuredCurrentPeak_A) -
                                       math::square(mParams.RTotal_ohm)));
            const float inductance_H = reactance_ohm / omega_rad_Hz;

            mParams.Ld_H = inductance_H;
            mParams.Lq_H = inductance_H;

            resetPhase();
            return true;
        }
        return false;
    }

    bool measurePsi(float Id_A, float Iq_A, float Uq_V, float omega_rad_Hz)
    {
        // Add safety check for zero speed to avoid division by zero
        if (std::abs(omega_rad_Hz) < 1.0f)
            return false;

        mUqSum += Uq_V;
        mIdSum += Id_A;
        mIqSum += Iq_A;
        mOmegaSum += omega_rad_Hz;

        if (++mSamples >= Config::PSI_MEASURE_SAMPLES)
        {
            float avgUq = mUqSum / mSamples;
            float avgId = mIdSum / mSamples;
            float avgIq = mIqSum / mSamples;
            float avgOmega = mOmegaSum / mSamples;

            // Calculation based on:
            // Uq = Rs * Iq + omega * (Ld * Id + Psi_pm)
            // Psi_pm = (Uq - Rs * Iq) / omega - Ld * Id
            mParams.flux_pm_Wb =
                (avgUq - (mParams.RTotal_ohm * avgIq)) / avgOmega - (mParams.Ld_H * avgId);

            // Safety: Ensure flux is positive.
            mParams.flux_pm_Wb = std::abs(mParams.flux_pm_Wb);

            return true;
        }
        return false;
    }

    void handleVoltageSearch()
    {
        if (++mTimer > 200)
        {
            mTimer = 0;
            if (mMeasuredCurrentPeak_A < (mTargetCurrent_A * 0.95f))
            {
                mInjectionVoltage_V += 0.05f;
                mMeasuredCurrentPeak_A = 0.0f;
            }
            else
            {
                mSamples = 1;
            }
        }
    }

    bool rampVoltageDown(float& UdInject_V)
    {
        constexpr float omegaInjection_rad_Hz = 2.0f * math::PI * Config::LS_INJECTION_FREQ_HZ;
        if (++mTimer > 200)
        {
            mTimer = 0;
            mInjectionVoltage_V = std::max(0.0f, mInjectionVoltage_V - 0.1f);
        }
        mTime_s += mPwmPeriod_s;
        UdInject_V = mInjectionVoltage_V * math::sin(omegaInjection_rad_Hz * mTime_s);
        return (mInjectionVoltage_V <= 0.0f);
    }

    bool averageOffset(float ref, float fb, float& result)
    {
        if (++mTimer < Config::ALIGN_SETTLE_SAMPLES)
            return false;
        float diff = ref - fb;
        // Wrap difference to [-PI, PI]
        while (diff > math::PI)
            diff -= 2.0f * math::PI;
        while (diff < -math::PI)
            diff += 2.0f * math::PI;

        mOffsetSum_rad += diff;
        if (++mSamples >= Config::ALIGN_SETTLE_SAMPLES)
        {
            result = mOffsetSum_rad / mSamples;
            return true;
        }
        return false;
    }

    void calculateFinalOffset()
    {
        float finalRad = (mOffsetFwd_rad + mOffsetBwd_rad) * 0.5f;
        float ticksPerRev = static_cast<float>(mParams.encoderTicks);
        float polePairs = static_cast<float>(mParams.polePairs);

        int32_t ticksPerElecRev = static_cast<int32_t>(ticksPerRev / polePairs);

        int32_t offset =
            static_cast<int32_t>((finalRad * ticksPerRev) / (math::TWO_PI * polePairs));

        offset %= ticksPerElecRev;
        if (offset < 0)
        {
            offset += ticksPerElecRev;
        }

        mParams.encoderOffset_ticks = offset;
    }

    void resetPhase()
    {
        mTimer = 0;
        mSamples = 0;
        mUdSum = 0;
        mUqSum = 0;
        mIdSum = 0;
        mIqSum = 0;
        mOmegaSum = 0;
        mMeasuredCurrentPeak_A = 0;
        mOffsetSum_rad = 0;
    }

    MotorParams& mParams;
    RampGenerator mRamp;
    RampGenerator mRampSpeed;
    State mState = State::IDLE;

    float mPwmPeriod_s;
    float mTargetCurrent_A{0};
    float mTime_s{0};
    float mInjectionVoltage_V{0.0f};
    float mMeasuredCurrentPeak_A{0};
    float mUdSum{0};
    float mUqSum{0};
    float mIdSum{0};
    float mIqSum{0};
    float mOmegaSum{0};
    float mOffsetFwd_rad{0.0f};
    float mOffsetBwd_rad{0.0f};
    float mOffsetSum_rad{0.0f};
    uint32_t mTimer{0};
    uint32_t mSamples{0};
};
} // namespace app