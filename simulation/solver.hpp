#pragma once
#include <functional>

namespace sim
{

struct MotorState
{
    float mId_A{0.0f};
    float mIq_A{0.0f};
    float mOmegaMech_rad_s{0.0f};
    float mThetaMech_rad{0.0f};

    MotorState operator+(const MotorState& o) const
    {
        return {mId_A + o.mId_A,
                mIq_A + o.mIq_A,
                mOmegaMech_rad_s + o.mOmegaMech_rad_s,
                mThetaMech_rad + o.mThetaMech_rad};
    }
    MotorState operator*(float dt) const
    {
        return {mId_A * dt, mIq_A * dt, mOmegaMech_rad_s * dt, mThetaMech_rad * dt};
    }
};

struct MotorInputs
{
    float mVd_V{0.0f};
    float mVq_V{0.0f};
    float mLoadTorque_Nm{0.0f};
};

class ISolver
{
  public:
    virtual ~ISolver() = default;

    using PhysicsFunction = std::function<MotorState(const MotorState&)>;

    virtual MotorState calculateNextState(const MotorState& currentState,
                                          float dt_s,
                                          const PhysicsFunction& getDerivatives) = 0;
};

class EulerSolver : public ISolver
{
  public:
    MotorState calculateNextState(const MotorState& currentState,
                                  float dt_s,
                                  const PhysicsFunction& getDerivatives) override
    {
        MotorState ratesOfChange = getDerivatives(currentState);
        return currentState + (ratesOfChange * dt_s);
    }
};

class RK4Solver : public ISolver
{
  public:
    MotorState calculateNextState(const MotorState& currentState,
                                  float dt_s,
                                  const PhysicsFunction& getDerivatives) override
    {
        // RK4 probes the derivatives at 4 points to find a better average slope
        MotorState k1 = getDerivatives(currentState);

        MotorState k2 = getDerivatives(currentState + (k1 * (dt_s * 0.5f)));

        MotorState k3 = getDerivatives(currentState + (k2 * (dt_s * 0.5f)));

        MotorState k4 = getDerivatives(currentState + (k3 * dt_s));

        MotorState averageSlope = (k1 + (k2 * 2.0f) + (k3 * 2.0f) + k4) * (1.0f / 6.0f);

        return currentState + (averageSlope * dt_s);
    }
};

} // namespace sim