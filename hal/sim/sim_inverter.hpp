class SimInverter : public app::IInverter
{
  public:
    void set_phase_voltages(float va, float vb, float vc, float vbus, bool isEnabled) override
    {
        mNextVu = isEnabled ? va : 0.0f;
        mNextVv = isEnabled ? vb : 0.0f;
        mNextVw = isEnabled ? vc : 0.0f;
    }

    void update_latched_voltages()
    {
        mCurrentVu = mNextVu;
        mCurrentVv = mNextVv;
        mCurrentVw = mNextVw;
    }

    float getVu() const
    {
        return mCurrentVu;
    }
    float getVv() const
    {
        return mCurrentVv;
    }
    float getVw() const
    {
        return mCurrentVw;
    }

  private:
    float mNextVu{0.0f}, mNextVv{0.0f}, mNextVw{0.0f};
    float mCurrentVu{0.0f}, mCurrentVv{0.0f}, mCurrentVw{0.0f};
};