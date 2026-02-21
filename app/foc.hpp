#pragma once
#include "interfaces.hpp"
#include <cstdint>

namespace app
{
class FOC
{
  public:
    explicit FOC(ICurrentSense& current_sense,
                 ITelemetry& telemetry,
                 IControlInputs& control_inputs)
        : current_sense(current_sense), telemetry(telemetry), control_inputs(control_inputs)
    {
    }

    void run_isr()
    {
        PhaseCurrentsRaw currents = current_sense.read_raw();
        ControlInputs inputs = control_inputs.read();
        isEnabled = inputs.enable;
        target_speed_rpm = inputs.target_speed_rpm;
        max_current_mA = inputs.max_current_mA;

        telemetry.publish5_i16(
            currents.ia_counts, currents.ib_counts, isEnabled, target_speed_rpm, max_current_mA);
    }

  private:
    ICurrentSense& current_sense;
    ITelemetry& telemetry;
    IControlInputs& control_inputs;
    bool isEnabled = false;
    int16_t target_speed_rpm = 0;
    int16_t max_current_mA = 0;
};
} // namespace app