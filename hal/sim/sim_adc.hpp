#pragma once
#include "interfaces.hpp"
#include "motor_model.hpp"

namespace sim
{

class SimADC : public app::IADC
{
  public:
    explicit SimADC(const MotorModel& motor) : mMotor(motor)
    {
    }

    app::PhaseCurrents read_amps() const override
    {
        PhaseCurrents simCurrents = mMotor.getPhaseCurrents();
        return {simCurrents.mIu_A, simCurrents.mIw_A};
    }

    float read_bus_voltage() const override
    {
        return mBusVoltage_V;
    }

    float read_temperature_celsius() const override
    {
        return 25.0f;
    }

    app::PhaseCurrentsRaw read_raw() const override
    {
        return {2048, 2048};
    }

    void calibrate_offset() override
    {
    }

    void setBusVoltage(float volts)
    {
        mBusVoltage_V = volts;
    }

  private:
    const MotorModel& mMotor;
    float mBusVoltage_V{24.0f};
};

} // namespace sim