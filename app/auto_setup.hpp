#pragma once
#include "foc.hpp"
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
        FINISHED
    };

    explicit AutoSetup(MotorParams& params, FOC& foc, float dt_s)
        : mParams(params), mFoc(foc), mRamp(dt_s), mDt_s(dt_s)
    {
    }

    std::tuple<float, float>
    run(float Id_A, float Iq_A, float UdApplied_V, float UqApplied_V, float UsLimit_V)
    {
        float IdRef_A = 0.0f;
        switch (mState)
        {
            case State::IDLE:
                mFoc.setCurrentControlGainsManual(0.5f, 0.01f);
                return {0.0f, 0.0f};

            case State::RS_RAMP_UP:
            {
                if (rampCurrent(mTargetCurrent_A, 2.0f, IdRef_A))
                {
                    mState = State::RS_MEASURE;
                }
                return mFoc.runCurrentControl(IdRef_A, 0.0f, Id_A, Iq_A, 0.0f, UsLimit_V, true);
            }

            case State::RS_MEASURE:
                if (measureRs(Id_A, Iq_A, UdApplied_V, 0.0f))
                {

                    mState = State::RS_RAMP_DOWN;
                }
                return mFoc.runCurrentControl(
                    mTargetCurrent_A, 0.0f, Id_A, Iq_A, 0.0f, UsLimit_V, true);

            case State::RS_RAMP_DOWN:
                if (rampCurrent(0.0f, 5.0f, IdRef_A))
                {
                    mState = State::LS_MEASURE;
                }
                return mFoc.runCurrentControl(IdRef_A, 0.0f, Id_A, Iq_A, 0.0f, UsLimit_V, true);

            case State::LS_MEASURE:
                if (measureLs(Id_A))
                {
                    mState = State::LS_RAMP_DOWN;
                }
                return {mLastUdCommand_V, 0.0f};

            case State::LS_RAMP_DOWN:
                if (rampVoltageDown())
                {
                    mState = State::PI_TUNE;
                }
                return {mLastUdCommand_V, 0.0f};

            case State::PI_TUNE:
                mFoc.setCurrentControlGains();
                mState = State::FINISHED;
                return {0.0f, 0.0f};

            case State::FINISHED:
            default:
                return {0.0f, 0.0f};
        }
    }

    void startAutoSetup(float IsAbs_A)
    {
        mTargetCurrent_A = IsAbs_A;
        mCurrentStep = 0;
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

    bool measureRs(float Id_A, float Iq_A, float Ud_V, float Uq_V)
    {
        if (++mTimer < 15000)
            return false;

        if (std::abs(Iq_A) > (mTargetCurrent_A * 0.05f))
        {
            resetPhase(); // Restart measurement if rotor moved
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

        mTime_s += mDt_s;
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
        mTime_s += mDt_s;
        mLastUdCommand_V = mInjectionVoltage_V * math::sin(omegaInjection_rad_Hz * mTime_s);
        return (mInjectionVoltage_V <= 0.0f);
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
    FOC& mFoc;
    RampGenerator mRamp;
    State mState = State::IDLE;

    uint8_t mCurrentStep{0};
    float mMeasuredI{};
    float mMeasuredU{};

    float mDt_s, mTargetCurrent_A{0}, mUdSum{0}, mIdSum{0}, mTime_s{0}, mMeasuredCurrentPeak_A{0};
    float mInjectionVoltage_V{0.0f};
    uint32_t mTimer{0}, mSamples{0};
    float mLastUdCommand_V{0.0f};
    float mLastUqCommand_V{0.0f};
    uint8_t mPiTuneState{0};
    uint32_t mPiTuneTimer{0};
    float mIdResponseMax{0.0f};
    float mIqResponseMax{0.0f};
};
} // namespace app