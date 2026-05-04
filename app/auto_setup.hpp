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
        FINISHED
    };

    explicit AutoSetup(MotorParams& params, float pwmPeriod_s)
        : mParams(params), mRamp(pwmPeriod_s), mRampSpeed(pwmPeriod_s), mPwmPeriod_s(pwmPeriod_s)
    {
    }

    std::tuple<float, float, float, float, bool, bool, float> step(float Id_A,
                                                                   float Iq_A,
                                                                   float UdApplied_V,
                                                                   float UqApplied_V,
                                                                   float thetaOpenLoop,
                                                                   float thetaEncoder)
    {
        float idRefOut_A = 0.0f;
        float iqRefOut_A = 0.0f;
        float udInjectOut_V = 0.0f;
        float uqInjectOut_V = 0.0f;
        bool bypassControl = false;
        bool triggerTune = false;
        float omegaRef_rad_Hz = 0.0f;

        switch (mState)
        {
            case State::IDLE:
                break;

            case State::RS_RAMP_UP:
            {
                if (rampCurrent(mTargetCurrent_A, 2.0f, idRefOut_A))
                {
                    mState = State::RS_MEASURE;
                }
                break;
            }

            case State::RS_MEASURE:
                idRefOut_A = mTargetCurrent_A;
                if (measureRs(Id_A, Iq_A, UdApplied_V, 0.0f))
                {
                    mState = State::RS_RAMP_DOWN;
                }
                break;

            case State::RS_RAMP_DOWN:
                if (rampCurrent(0.0f, 5.0f, idRefOut_A))
                {
                    mState = State::LS_MEASURE;
                }
                break;

            case State::LS_MEASURE:
                bypassControl = true;
                if (measureLs(Id_A))
                {
                    mState = State::LS_RAMP_DOWN;
                }
                udInjectOut_V = mLastUdCommand_V;
                break;

            case State::LS_RAMP_DOWN:
                bypassControl = true;
                if (rampVoltageDown())
                {
                    mState = State::PI_TUNE;
                }
                udInjectOut_V = mLastUdCommand_V;
                break;

            case State::PI_TUNE:
                triggerTune = true;
                bypassControl = false;
                resetPhase();
                mState = State::ALIGN_FWD;
                break;

            case State::ALIGN_FWD:
                triggerTune = false;
                bypassControl = false;
                udInjectOut_V = 0.0f;
                udInjectOut_V = 0.0f;
                idRefOut_A = mTargetCurrent_A;
                omegaRef_rad_Hz = mRampSpeed.update(mAlignSpeed_rad_Hz, 150.0f);
                mOmegaDebug_rad_Hz = omegaRef_rad_Hz;
                if (std::abs(omegaRef_rad_Hz - mAlignSpeed_rad_Hz) < 0.1f)
                {
                    if (averageOffset(thetaOpenLoop, thetaEncoder, mOffsetFwd))
                    {
                        mState = State::ALIGN_BWD;
                        resetPhase();
                    }
                }
                break;

            case State::ALIGN_BWD:
                idRefOut_A = mTargetCurrent_A;
                omegaRef_rad_Hz = mRampSpeed.update(-mAlignSpeed_rad_Hz, 150.0f);
                mOmegaDebug_rad_Hz = omegaRef_rad_Hz;
                if (std::abs(omegaRef_rad_Hz + mAlignSpeed_rad_Hz) < 0.1f)
                {
                    if (averageOffset(thetaOpenLoop, thetaEncoder, mOffsetBwd))
                    {
                        calculateFinalOffset();
                        mState = State::FINISHED;
                        resetPhase();
                    }
                }
                break;

            case State::FINISHED:
            default:
                break;
        }

        return {idRefOut_A,
                iqRefOut_A,
                udInjectOut_V,
                uqInjectOut_V,
                bypassControl,
                triggerTune,
                omegaRef_rad_Hz};
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

    float debug() const
    {
        return mOmegaDebug_rad_Hz;
    }

  private:
    bool rampCurrent(float target_A, float rampRate_A_Hz, float& id_ref_out_A)
    {
        id_ref_out_A = mRamp.update(target_A, rampRate_A_Hz);
        return std::abs(id_ref_out_A - target_A) < 0.01f;
    }

    bool measureRs(float Id_A, float Iq_A, float Ud_V, float Uq_V)
    {
        if (++mTimer < 15000)
            return false;

        if (std::abs(Iq_A) > (mTargetCurrent_A * 0.05f))
        {
            resetPhase();
            return false;
        }

        mUdSum += Ud_V;
        mIdSum += Id_A;

        if (++mSamples >= 10000)
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

    bool measureLs(float Id_A)
    {
        constexpr float omega_rad_Hz = 2.0f * math::PI * 1000.0f;

        if (mInjectionVoltage_V == 0.0f)
        {
            mInjectionVoltage_V = mParams.RTotal_ohm * mTargetCurrent_A;
        }

        mTime_s += mPwmPeriod_s;
        mLastUdCommand_V = mInjectionVoltage_V * math::sin(omega_rad_Hz * mTime_s);
        mMeasuredCurrentPeak_A = std::max(mMeasuredCurrentPeak_A, std::abs(Id_A));

        if (mSamples == 0)
        {
            handleVoltageSearch();
            return false;
        }

        if (++mSamples > 10000)
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

    bool rampVoltageDown()
    {
        constexpr float omegaInjection_rad_Hz = 2.0f * math::PI * 1000.0f;
        if (++mTimer > 200)
        {
            mTimer = 0;
            mInjectionVoltage_V = std::max(0.0f, mInjectionVoltage_V - 0.1f);
        }
        mTime_s += mPwmPeriod_s;
        mLastUdCommand_V = mInjectionVoltage_V * math::sin(omegaInjection_rad_Hz * mTime_s);
        return (mInjectionVoltage_V <= 0.0f);
    }

    bool averageOffset(float ref, float fb, float& result)
    {
        if (++mTimer < 5000)
            return false;
        float diff = ref - fb;
        // Wrap difference to [-PI, PI]
        while (diff > math::PI)
            diff -= 2.0f * math::PI;
        while (diff < -math::PI)
            diff += 2.0f * math::PI;

        mOffsetSum += diff;
        if (++mSamples >= 10000)
        {
            result = mOffsetSum / mSamples;
            return true;
        }
        return false;
    }

    void calculateFinalOffset()
    {
        float finalRad = (mOffsetFwd + mOffsetBwd) * 0.5f;
        // Convert radians to encoder ticks
        float ticksPerRev = static_cast<float>(mParams.encoderTicks);
        float polePairs = static_cast<float>(mParams.polePairs);
        mParams.encoderOffset_ticks =
            static_cast<int32_t>((finalRad * ticksPerRev) / (2.0f * math::PI * polePairs));
    }

    void resetPhase()
    {
        mTimer = 0;
        mSamples = 0;
        mUdSum = 0;
        mIdSum = 0;
        mMeasuredCurrentPeak_A = 0;
    }

    MotorParams& mParams;
    RampGenerator mRamp;
    RampGenerator mRampSpeed;
    State mState = State::IDLE;

    float mPwmPeriod_s;
    float mTargetCurrent_A{0};
    float mUdSum{0};
    float mIdSum{0};
    float mTime_s{0};
    float mMeasuredCurrentPeak_A{0};
    float mInjectionVoltage_V{0.0f};
    uint32_t mTimer{0}, mSamples{0};
    float mLastUdCommand_V{0.0f};
    //
    float mOffsetFwd{0.0f};
    float mOffsetBwd{0.0f};
    float mOffsetSum{0.0f};
    float mAlignSpeed_rad_Hz{40.0f};
    float thetaOpenLoop{0.0f};
    float thetaEncoder{0.0f};

    float mOmegaDebug_rad_Hz{0.0f};
};
} // namespace app