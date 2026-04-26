#pragma once
#include "foc.hpp" // Added to access FOC
#include "math.hpp"
#include "motor_params.hpp"
#include "ramp_generator.hpp"
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
        FINISHED
    };

    explicit AutoSetup(MotorParams& params, FOC& foc, float dt_s)
        : mParams(params), mFoc(foc), mRamp(dt_s), mDt_s(dt_s)
    {
    }

    std::tuple<float, float> run(float Id_A, float Iq_A, float UdApplied_V, float UsLimit_V)
    {
        float IdRef_A = 0.0f;
        switch (mState)
        {
            case State::RS_RAMP_UP:
                if (rampCurrent(mTargetCurrent_A, 2.0f, IdRef_A))
                {
                    mState = State::RS_MEASURE;
                }
                return mFoc.runCurrentControl(IdRef_A, 0.0f, Id_A, Iq_A, 0.0f, UsLimit_V, true);

            case State::RS_MEASURE:
                if (measureRs(Id_A, UdApplied_V))
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
                    mState = State::FINISHED;
                }
                return {mLastUdCommand_V, 0.0f};

            default:
                return {0.0f, 0.0f};
        }
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

  private:
    bool rampCurrent(float target_A, float rampRate_A_Hz, float& id_ref_out_A)
    {
        id_ref_out_A = mRamp.update(target_A, rampRate_A_Hz);
        return std::abs(id_ref_out_A - target_A) < 0.01f;
    }

    bool measureRs(float Id_A, float Ud_V)
    {

        if (++mTimer < 2000)
            return false;

        mUdSum += Ud_V;
        mIdSum += Id_A;

        if (++mSamples >= 2000)
        {

            mParams.Rs_ohm = mUdSum / mIdSum;
            resetPhase();
            return true;
        }
        return false;
    }

    bool rampVoltageDown()
    {
        constexpr float omegaInjection_rad_Hz = math::TWO_PI * 500.0f;

        if (++mTimer > 200)
        {
            mTimer = 0;
            mInjectionVoltage_V = std::max(0.0f, mInjectionVoltage_V - 0.1f);
        }

        mTime_s += mDt_s;
        mLastUdCommand_V = mInjectionVoltage_V * math::sin(omegaInjection_rad_Hz * mTime_s);

        return (mInjectionVoltage_V <= 0.0f); // Finished when voltage is gone
    }

    bool measureLs(float Id_A)
    {
        constexpr float omega_rad_Hz = math::TWO_PI * 500.0f;

        if (mInjectionVoltage_V == 0.0f)
        {
            mInjectionVoltage_V = mParams.Rs_ohm * mTargetCurrent_A;
        }

        mTime_s += mDt_s;
        mLastUdCommand_V = mInjectionVoltage_V * math::sin(omega_rad_Hz * mTime_s);
        mMeasuredCurrentPeak_A = std::max(mMeasuredCurrentPeak_A, std::abs(Id_A));

        if (mSamples == 0)
        {
            handleVoltageSearch();
            return false;
        }

        if (++mSamples > 4000)
        {
            const float reactance_ohm =
                std::sqrt(std::max(0.0f,
                                   math::square(mInjectionVoltage_V / mMeasuredCurrentPeak_A) -
                                       math::square(mParams.Rs_ohm)));
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

    void resetPhase()
    {
        mTimer = 0;
        mSamples = 0;
        mUdSum = 0;
        mIdSum = 0;
        mMeasuredCurrentPeak_A = 0;
        mDecayCounter_ticks = 0;
    }

    MotorParams& mParams;
    FOC& mFoc;
    RampGenerator mRamp;
    State mState = State::IDLE;
    float mDt_s, mTargetCurrent_A{0}, mUdSum{0}, mIdSum{0}, mTime_s{0}, mMeasuredCurrentPeak_A{0};
    float mInjectionVoltage_V{0.0f};
    uint32_t mTimer{0}, mSamples{0}, mDecayCounter_ticks{0};
    float mLastUdCommand_V{0.0f};
};
} // namespace app