#pragma once
#include "foc.hpp"
#include "interfaces.hpp"

namespace app
{
class MotorControl
{
  public:
    MotorControl(ICurrentSense& current_sense,
                 ITelemetry& telemetry,
                 IControlInputs& control_inputs)
        : foc(current_sense, telemetry, control_inputs)
    {
    }

    void run_isr()
    {
        foc.run_isr();
    }

  private:
    FOC foc;
};

} // namespace app