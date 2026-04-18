#pragma once
#include "load.hpp"
#include "math.hpp"
#include "motor_parameter_sets.hpp"
#include "solver.hpp"
#include "transform.hpp"

namespace sim
{

struct PhaseCurrents
{
    float mIu_A{0.0f};
    float mIv_A{0.0f};
    float mIw_A{0.0f};
};

class MotorModel
{
  public:
    explicit MotorModel(const SimMotorParams& params, ISolver& solver, const ILoadModel& load)
        : mParams(params), mSolver(solver), mLoad(load)
    {
    }

    void stepSimulation(float dt_s, float vu_V, float vv_V, float vw_V)
    {
        auto [alphaV, betaV] = Transforms::clarke(vu_V, vv_V, vw_V);

        mState = mSolver.calculateNextState(
            mState,
            dt_s,
            [&](const MotorState& state)
            {
                float subThetaElec = state.mThetaMech_rad * mParams.polePairs_count;
                auto [vd, vq] = Transforms::park(alphaV, betaV, subThetaElec);

                MotorState rates;
                float omegaElec = state.mOmegaMech_rad_s * mParams.polePairs_count;

                // Physics...
                rates.mId_A =
                    (vd - mParams.Rs_ohm * state.mId_A + omegaElec * mParams.Lq_H * state.mIq_A) /
                    mParams.Ld_H;
                rates.mIq_A = (vq - mParams.Rs_ohm * state.mIq_A -
                               omegaElec * (mParams.Ld_H * state.mId_A + mParams.fluxPm_Wb)) /
                              mParams.Lq_H;

                float torqueGen = 1.5f * mParams.polePairs_count * mParams.fluxPm_Wb * state.mIq_A;

                // PULL FROM CONNECTED LOAD OBJECT
                float currentLoad = mLoad.getTorque_Nm(state.mOmegaMech_rad_s);

                rates.mOmegaMech_rad_s = (torqueGen - currentLoad) / mParams.inertia_kgm2;
                rates.mThetaMech_rad = state.mOmegaMech_rad_s;

                return rates;
            });

        if (mState.mThetaMech_rad > math::PI)
            mState.mThetaMech_rad -= math::TWO_PI;
        if (mState.mThetaMech_rad <= -math::PI)
            mState.mThetaMech_rad += math::TWO_PI;
    }

    PhaseCurrents getPhaseCurrents() const
    {
        float thetaElec = mState.mThetaMech_rad * mParams.polePairs_count;

        auto [alphaI, betaI] = Transforms::inversePark(mState.mId_A, mState.mIq_A, thetaElec);
        auto [u, v, w] = Transforms::inverseClarke(alphaI, betaI);

        return {u, v, w};
    }

    MotorState getState() const
    {
        return mState;
    }

  private:
    MotorState calculateDerivatives(const MotorState& state, const MotorInputs& inputs) const
    {
        MotorState rates;
        float omegaElec = state.mOmegaMech_rad_s * mParams.polePairs_count;

        rates.mId_A =
            (inputs.mVd_V - mParams.Rs_ohm * state.mId_A + omegaElec * mParams.Lq_H * state.mIq_A) /
            mParams.Ld_H;

        rates.mIq_A = (inputs.mVq_V - mParams.Rs_ohm * state.mIq_A -
                       omegaElec * (mParams.Ld_H * state.mId_A + mParams.fluxPm_Wb)) /
                      mParams.Lq_H;

        float torque_Nm = 1.5f * mParams.polePairs_count *
                          (mParams.fluxPm_Wb * state.mIq_A +
                           (mParams.Ld_H - mParams.Lq_H) * state.mId_A * state.mIq_A);

        rates.mOmegaMech_rad_s = (torque_Nm - inputs.mLoadTorque_Nm -
                                  (mParams.viscousFriction_Nms * state.mOmegaMech_rad_s)) /
                                 mParams.inertia_kgm2;

        rates.mThetaMech_rad = state.mOmegaMech_rad_s;

        return rates;
    }

    SimMotorParams mParams;
    ISolver& mSolver;
    MotorState mState;
    MotorInputs mLastInputs;
    const ILoadModel& mLoad;
};

} // namespace sim