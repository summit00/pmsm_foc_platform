#pragma once
#include "drive_state_machine.hpp"

/**
 * @file control_drive_hardware.hpp
 * @brief Concrete implementation of IDriveHardware mapping Drive State Machine actions.
 */

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
    /**
     * @brief Construct a new Control Drive Hardware object.
     * @param parent Reference to the parent Control class instance.
     */
    explicit ControlDriveHardware(Control& parent);

    /**
     * @brief Enable the gate driver hardware.
     */
    void enableGateDriver() override;

    /**
     * @brief Disable the gate driver hardware.
     */
    void disableGateDriver() override;

    /**
     * @brief Enable PWM outputs.
     */
    void enablePwm() override;

    /**
     * @brief Disable PWM outputs.
     */
    void disablePwm() override;

    /**
     * @brief Reset internal controllers (FOC current loop, state estimators, etc.).
     */
    void resetControllers() override;

    /**
     * @brief Check if the underlying hardware (e.g. ADCs, encoders) is ready.
     * @return true if hardware is ready, false otherwise.
     */
    bool isHardwareReady() const override;

    /**
     * @brief Check if the gate driver is powered on and ready.
     * @return true if the gate driver is ready, false otherwise.
     */
    bool isGateDriverReady() const override;

    /**
     * @brief Check if any active hardware faults exist (e.g. overcurrent, overvoltage).
     * @return true if an active fault exists, false otherwise.
     */
    bool hasActiveFault() const override;

  private:
    Control& mParent; ///< Reference to the parent Control instance.
};

} // namespace app
