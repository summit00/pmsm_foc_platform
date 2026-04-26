#pragma once
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
        MEASURE_L, // Combined inductance identification
        FINISHED
    };

    explicit AutoSetup(MotorParams& params, float dt_s) : mParams(params), mRamp(dt_s), mDt(dt_s)
    {
    }

    std::tuple<float, float, bool> run(float id_meas, float iq_meas, float ud_meas)
    {
        switch (mState)
        {
            case State::MEASURE_RS:
                return {runRs(id_meas, ud_meas), 0.0f, false};

            case State::MEASURE_L:
                // We inject only into the D-axis for general inductance
                return {runInductance(id_meas), 0.0f, true};

            default:
                return {0.0f, 0.0f, false};
        }
    }

    void start(float testCurrent_A)
    {
        mTargetCurrent_A = testCurrent_A;
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
    float runRs(float id_meas, float ud_meas)
    {
        float ref = mRamp.update(mTargetCurrent_A, 2.0f);
        if (std::abs(ref - mTargetCurrent_A) < 0.01f)
        {
            if (++mTimer > 1000)
            {
                mUdSum += ud_meas;
                mIdSum += id_meas;
                if (++mSamples >= 1000)
                {
                    mParams.Rs_ohm = mUdSum / mIdSum;
                    mState = State::MEASURE_L; // Transition to single L measurement
                    resetPhase();
                }
            }
        }
        return ref;
    }

    float runInductance(float i_meas)
    {
        constexpr float f_inj = 500.0f;
        constexpr float w_inj = 2.0f * math::PI * f_inj;

        // Start with a voltage guess based on Rs
        if (mVoltageAmplitude == 0.0f)
        {
            mVoltageAmplitude = mParams.Rs_ohm * mTargetCurrent_A;
        }

        mTime += mDt;
        float v_cmd = mVoltageAmplitude * math::sin(w_inj * mTime);
        mCurrentPk = std::max(mCurrentPk, std::abs(i_meas));

        // Adaptive search for target current amplitude
        if (++mSettleCount > 200)
        {
            mSettleCount = 0;
            if (mCurrentPk < (mTargetCurrent_A * 0.95f))
            {
                mVoltageAmplitude += 0.05f;
                mCurrentPk = 0;
                mSamples = 0;
            }
            else
            {
                if (++mSamples > 4000)
                {
                    float z_mag = mVoltageAmplitude / mCurrentPk;
                    float r_sq = mParams.Rs_ohm * mParams.Rs_ohm;
                    float L = std::sqrt(std::max(0.0f, z_mag * z_mag - r_sq)) / w_inj;

                    // Apply identified L to both D and Q axes
                    mParams.Ld_H = L;
                    mParams.Lq_H = L;

                    mState = State::FINISHED;
                    resetPhase();
                }
            }
        }
        return v_cmd;
    }

    void resetPhase()
    {
        mTimer = 0;
        mSamples = 0;
        mUdSum = 0;
        mIdSum = 0;
        mTime = 0;
        mCurrentPk = 0;
        mVoltageAmplitude = 0.0f;
        mSettleCount = 0;
    }

    MotorParams& mParams;
    RampGenerator mRamp;
    State mState = State::IDLE;
    float mDt, mTargetCurrent_A{0}, mUdSum{0}, mIdSum{0}, mTime{0}, mCurrentPk{0};
    float mVoltageAmplitude{0.0f};
    uint32_t mTimer{0}, mSamples{0}, mSettleCount{0};
};
} // namespace app