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
        MEASURE_RS,
        MEASURE_LS,
        FINISHED
    };

    explicit AutoSetup(MotorParams& params, FOC& foc, float dt_s)
        : mParams(params), mFoc(foc), mRamp(dt_s), mDt_s(dt_s)
    {
    }

    std::tuple<float, float> run(float Id_A, float Iq_A, float UdApplied_V, float UsLimit_V)
    {
        switch (mState)
        {
            case State::MEASURE_RS:
            {
                float IdRef_A = measureRs(Id_A, UdApplied_V);

                auto [ud, uq] =
                    mFoc.runCurrentControl(IdRef_A, 0.0f, Id_A, Iq_A, 0.0f, UsLimit_V, true);
                return {ud, uq};
            }

            case State::MEASURE_LS:
            {
                float ud_inj = measureLs(Id_A);
                return {ud_inj, 0.0f};
            }

            default:
                return {0.0f, 0.0f};
        }
    }

    void startAutoSetup(float IsAbs_A)
    {
        mTargetCurrent_A = IsAbs_A;
        mState = State::MEASURE_RS;
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
    float measureRs(float Id_A, float Ud_V)
    {
        float IdRef_A = mRamp.update(mTargetCurrent_A, 2.0f);
        if (std::abs(IdRef_A - mTargetCurrent_A) < 0.01f)
        {
            if (++mTimer > 1000)
            {
                mUdSum += Ud_V;
                mIdSum += Id_A;
                if (++mSamples >= 1000)
                {
                    mParams.Rs_ohm = mUdSum / mIdSum;
                    mState = State::MEASURE_LS;
                    resetPhase();
                }
            }
        }
        return IdRef_A;
    }

    float measureLs(float Id_A)
    {
        constexpr float injectionFreq_Hz = 500.0f;
        constexpr float omegaInjection_rad_Hz = math::TWO_PI * injectionFreq_Hz;

        if (mInjectionVoltage_V == 0.0f)
        {
            mInjectionVoltage_V = mParams.Rs_ohm * mTargetCurrent_A;
        }

        if (mDecayCounter_ticks < 2000)
        {
            mDecayCounter_ticks++;
            mMeasuredCurrentPeak_A = 0.0f;
            return 0.0f;
        }

        mTime_s += mDt_s;
        const float ud_command_V = mInjectionVoltage_V * math::sin(omegaInjection_rad_Hz * mTime_s);
        mMeasuredCurrentPeak_A = std::max(mMeasuredCurrentPeak_A, std::abs(Id_A));

        if (mSamples == 0)
        {
            handleVoltageSearch();
            return ud_command_V;
        }

        if (++mSamples > 4000)
        {
            // Calculate Impedance Magnitude (|Z| = V / I)
            // Pythagorean Theorem: L = sqrt(Z^2 - R^2) / omega
            const float reactance_ohm =
                std::sqrt(std::max(0.0f,
                                   math::square(mInjectionVoltage_V / mMeasuredCurrentPeak_A) -
                                       math::square(mParams.Rs_ohm)));
            const float inductance_H = reactance_ohm / omegaInjection_rad_Hz;

            mParams.Ld_H = inductance_H;
            mParams.Lq_H = inductance_H;

            mState = State::FINISHED;
            resetPhase();
        }

        return ud_command_V;
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
        mTime_s = 0;
        mMeasuredCurrentPeak_A = 0;
        mInjectionVoltage_V = 0.0f;
        mDecayCounter_ticks = 0;
    }

    MotorParams& mParams;
    FOC& mFoc;
    RampGenerator mRamp;
    State mState = State::IDLE;
    float mDt_s, mTargetCurrent_A{0}, mUdSum{0}, mIdSum{0}, mTime_s{0}, mMeasuredCurrentPeak_A{0};
    float mInjectionVoltage_V{0.0f};
    uint32_t mTimer{0}, mSamples{0}, mDecayCounter_ticks{0};
};
} // namespace app