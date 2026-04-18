namespace sim
{
class ILoadModel
{
  public:
    virtual ~ILoadModel() = default;
    virtual float getTorque_Nm(float currentOmegaMech_rad_s) const = 0;
};

class SimpleLoad : public ILoadModel
{
  public:
    void setBrakingTorque(float torque_Nm)
    {
        mTorque_Nm = torque_Nm;
    }
    float getTorque_Nm(float /*speed*/) const override
    {
        return mTorque_Nm;
    }

  private:
    float mTorque_Nm{0.0f};
};
} // namespace sim