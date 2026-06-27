#pragma once
#include "math.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace app
{

class BodeSweeper
{
  public:
    static constexpr size_t MAX_POINTS = 32;

    enum class State
    {
        IDLE,
        SETTLING,
        MEASURING,
        DONE
    };

    explicit BodeSweeper(float ctrlPeriod_s) : mCtrlPeriod_s(ctrlPeriod_s)
    {
    }

    void configure(const float* frequencies, size_t numFrequencies, float amplitude)
    {
        mNumPoints = std::min(numFrequencies, MAX_POINTS);
        for (size_t i = 0; i < mNumPoints; ++i)
        {
            mSweepFrequencies[i] = frequencies[i];
            mMagnitudes[i] = 0.0f;
            mPhases[i] = 0.0f;
            mClosedLoopMagnitudes[i] = 0.0f;
            mClosedLoopPhases[i] = 0.0f;
        }
        mAmplitude = amplitude;
        reset();
    }

    void reset()
    {
        mState = State::IDLE;
        mCurrentFreqIdx = 0;
        mTheta = 0.0f;
        resetAccumulators();
    }

    void start()
    {
        if (mNumPoints == 0)
        {
            mState = State::DONE;
            return;
        }
        mState = State::SETTLING;
        mCurrentFreqIdx = 0;
        mTheta = 0.0f;
        mTickCounter = 0;
        prepareFrequencyStep();
    }

    float step(float u_pi, float measured)
    {
        if (mState == State::IDLE || mState == State::DONE)
        {
            return 0.0f;
        }

        float perturbation = mAmplitude * math::sin(mTheta);

        // Core system parameter mapping
        float inputSignal = u_pi + perturbation; // Total input to plant (u + Δu)
        float outputSignal = measured;           // Measured response (y)

        // State Machine execution logic
        if (mState == State::SETTLING)
        {
            if (++mTickCounter >= mSettleTicks)
            {
                mState = State::MEASURING;
                mTickCounter = 0;
                resetAccumulators();
            }
        }
        else if (mState == State::MEASURING)
        {
            float sinTheta = math::sin(mTheta);
            float cosTheta = math::cos(mTheta);

            // Integrate Fourier components over designated step cycle periods
            mInSinSum += inputSignal * sinTheta;
            mInCosSum += inputSignal * cosTheta;
            mOutSinSum += outputSignal * sinTheta;
            mOutCosSum += outputSignal * cosTheta;

            // Isolate individual controller component path metrics
            mControllerSinSum += u_pi * sinTheta;
            mControllerCosSum += u_pi * cosTheta;

            if (++mTickCounter >= mMeasureTicks)
            {
                float plant_denom = math::square(mInSinSum) + math::square(mInCosSum);
                float ctrl_denom = math::square(mOutSinSum) + math::square(mOutCosSum);

                if (plant_denom > 1e-12f && ctrl_denom > 1e-12f)
                {
                    // Complex Division: Plant P = Measured / U_total
                    float P_real =
                        ((mOutSinSum * mInSinSum) + (mOutCosSum * mInCosSum)) / plant_denom;
                    float P_imag =
                        ((mOutCosSum * mInSinSum) - (mOutSinSum * mInCosSum)) / plant_denom;

                    // Complex Division: Controller C = - (U_pi / Measured)
                    float C_real =
                        -((mControllerSinSum * mOutSinSum) + (mControllerCosSum * mOutCosSum)) /
                        ctrl_denom;
                    float C_imag =
                        -((mControllerCosSum * mOutSinSum) - (mControllerSinSum * mOutCosSum)) /
                        ctrl_denom;

                    // Compute Combined Open Loop system response: G_open = C * P
                    float openLoop_real = (C_real * P_real) - (C_imag * P_imag);
                    float openLoop_imag = (C_real * P_imag) + (C_imag * P_real);

                    // Compute and assign Open Loop Polar values
                    mMagnitudes[mCurrentFreqIdx] =
                        std::sqrt(math::square(openLoop_real) + math::square(openLoop_imag));
                    mPhases[mCurrentFreqIdx] =
                        std::atan2(openLoop_imag, openLoop_real) * (180.0f / math::PI);

                    // Compute Closed Loop tracking profile: T = G_open / (1 + G_open)
                    float cl_num_real = openLoop_real;
                    float cl_num_imag = openLoop_imag;
                    float cl_den_real = 1.0f + openLoop_real;
                    float cl_den_imag = openLoop_imag;

                    float cl_denom = math::square(cl_den_real) + math::square(cl_den_imag);

                    if (cl_denom > 1e-12f)
                    {
                        float cl_real =
                            ((cl_num_real * cl_den_real) + (cl_num_imag * cl_den_imag)) / cl_denom;
                        float cl_imag =
                            ((cl_num_imag * cl_den_real) - (cl_num_real * cl_den_imag)) / cl_denom;

                        mClosedLoopMagnitudes[mCurrentFreqIdx] =
                            std::sqrt(math::square(cl_real) + math::square(cl_imag));
                        mClosedLoopPhases[mCurrentFreqIdx] =
                            std::atan2(cl_imag, cl_real) * (180.0f / math::PI);
                    }
                }

                // Advance frequency pointer
                mCurrentFreqIdx++;
                if (mCurrentFreqIdx >= mNumPoints)
                {
                    mState = State::DONE;
                }
                else
                {
                    mState = State::SETTLING;
                    mTickCounter = 0;
                    prepareFrequencyStep();
                }
            }
        }

        // Advance phase angle accumulator for the next loop execution step
        if (mState != State::DONE)
        {
            float freq = mSweepFrequencies[mCurrentFreqIdx];
            mTheta += math::TWO_PI * freq * mCtrlPeriod_s;
            if (mTheta >= math::TWO_PI)
            {
                mTheta -= math::TWO_PI;
            }
        }

        return perturbation;
    }

    State getState() const
    {
        return mState;
    }

    const float* getSweepFrequencies() const
    {
        return mSweepFrequencies;
    }
    const float* getMagnitudes() const
    {
        return mMagnitudes;
    }
    const float* getPhases() const
    {
        return mPhases;
    }
    const float* getClosedLoopMagnitudes() const
    {
        return mClosedLoopMagnitudes;
    }
    const float* getClosedLoopPhases() const
    {
        return mClosedLoopPhases;
    }
    size_t getNumPoints() const
    {
        return mNumPoints;
    }

  private:
    void prepareFrequencyStep()
    {
        float freq = mSweepFrequencies[mCurrentFreqIdx];
        // Settle for at least 10 cycles or 20 ms
        float cycles = 10.0f;
        float minTime = 0.02f;
        float time = std::max(cycles / freq, minTime);
        mSettleTicks = static_cast<uint32_t>(time / mCtrlPeriod_s);

        // Measure for at least 50 cycles or 200 ms to get clean integration/DFT
        float measureCycles = 50.0f;
        float minMeasureTime = 0.2f;
        float measureTime = std::max(measureCycles / freq, minMeasureTime);
        mMeasureTicks = static_cast<uint32_t>(measureTime / mCtrlPeriod_s);
    }

    void resetAccumulators()
    {
        mInSinSum = 0.0f;
        mInCosSum = 0.0f;
        mOutSinSum = 0.0f;
        mOutCosSum = 0.0f;
        mControllerSinSum = 0.0f;
        mControllerCosSum = 0.0f;
    }

    float mCtrlPeriod_s;
    float mAmplitude{0.0f};
    State mState{State::IDLE};
    size_t mCurrentFreqIdx{0};
    float mTheta{0.0f};
    uint32_t mTickCounter{0};
    uint32_t mSettleTicks{0};
    uint32_t mMeasureTicks{0};
    float mInSinSum{0.0f};
    float mInCosSum{0.0f};
    float mOutSinSum{0.0f};
    float mOutCosSum{0.0f};
    float mControllerSinSum{0.0f};
    float mControllerCosSum{0.0f};
    float mSweepFrequencies[MAX_POINTS] = {};
    float mMagnitudes[MAX_POINTS] = {};
    float mPhases[MAX_POINTS] = {};
    float mClosedLoopMagnitudes[MAX_POINTS] = {};
    float mClosedLoopPhases[MAX_POINTS] = {};
    size_t mNumPoints{0};
};

} // namespace app
