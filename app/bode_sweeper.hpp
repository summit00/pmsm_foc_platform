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

    // Call this at the control loop rate (e.g. 20kHz for current loop, or 2kHz for speed loop)
    // Returns the injection signal (perturbation) to add to the reference
    float step(float inputSignal, float outputSignal)
    {
        if (mState == State::IDLE || mState == State::DONE)
        {
            return 0.0f;
        }

        // Generate perturbation using current theta
        float perturbation = mAmplitude * math::sin(mTheta);

        // State Machine & Correlation (uses the current theta that generated the perturbation)
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

            // Correlate input and output with sine/cos at the excitation frequency
            mInSinSum += inputSignal * sinTheta;
            mInCosSum += inputSignal * cosTheta;
            mOutSinSum += outputSignal * sinTheta;
            mOutCosSum += outputSignal * cosTheta;

            if (++mTickCounter >= mMeasureTicks)
            {
                // Compute transfer function G = Output / Input
                float inMagSq = math::square(mInSinSum) + math::square(mInCosSum);
                if (inMagSq > 1e-12f)
                {
                    float outMag = std::sqrt(math::square(mOutSinSum) + math::square(mOutCosSum));
                    float inMag = std::sqrt(inMagSq);
                    float magnitude = outMag / inMag;

                    float phaseIn = std::atan2(mInCosSum, mInSinSum);
                    float phaseOut = std::atan2(mOutCosSum, mOutSinSum);
                    float phaseDiff = phaseOut - phaseIn;

                    // Wrap to [-pi, pi]
                    while (phaseDiff > math::PI)
                        phaseDiff -= math::TWO_PI;
                    while (phaseDiff < -math::PI)
                        phaseDiff += math::TWO_PI;

                    float phaseDeg = phaseDiff * (180.0f / math::PI);

                    mMagnitudes[mCurrentFreqIdx] = magnitude;
                    mPhases[mCurrentFreqIdx] = phaseDeg;
                }
                else
                {
                    mMagnitudes[mCurrentFreqIdx] = 0.0f;
                    mPhases[mCurrentFreqIdx] = 0.0f;
                }

                // Advance to next frequency
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

        // Update phase angle for the NEXT step
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
    }

    float mCtrlPeriod_s;
    float mAmplitude{0.0f};

    State mState{State::IDLE};
    size_t mCurrentFreqIdx{0};
    float mTheta{0.0f};
    uint32_t mTickCounter{0};
    uint32_t mSettleTicks{0};
    uint32_t mMeasureTicks{0};

    // Accumulators for correlation/DFT
    float mInSinSum{0.0f};
    float mInCosSum{0.0f};
    float mOutSinSum{0.0f};
    float mOutCosSum{0.0f};

    // Result arrays (flat memory layout, no dynamic allocation)
    float mSweepFrequencies[MAX_POINTS] = {};
    float mMagnitudes[MAX_POINTS] = {};
    float mPhases[MAX_POINTS] = {};
    size_t mNumPoints{0};
};

} // namespace app
