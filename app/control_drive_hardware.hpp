#pragma once
#include "drive_state_machine.hpp"

namespace app
{

class Control; // Forward declaration

/**
 * @brief Concrete implementation of IDriveHardware mapping the Drive State Machine
 *        actions to the hardware-dependent control classes.
 */
class ControlDriveHardware : public IDriveHardware
{
  public:
    explicit ControlDriveHardware(Control& parent);

    void enableGateDriver() override;
    void disableGateDriver() override;
    void enablePwm() override;
    void disablePwm() override;
    void resetControllers() override;

    bool isHardwareReady() const override;
    bool isGateDriverReady() const override;
    bool hasActiveFault() const override;

  private:
    Control& mParent;
};

} // namespace app
