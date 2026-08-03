#pragma once
#include <cstdint>

namespace app
{

/**
 * @brief States representing the drive lifecycle.
 */
enum class DriveState : uint8_t
{
    PowerOff = 0, ///< Device is powered off or uninitialized.
    Init,         ///< Device is initializing and running self-checks.
    Ready,        ///< Device is ready to enable the power stage.
    Enabling,     ///< Enabling the gate driver and waiting for status.
    Enabled,      ///< Power stage is active and inverter is producing voltage.
    Fault         ///< Safety fault state; power stage is disabled.
};

/**
 * @brief Hardware abstraction interface for the Drive State Machine.
 */
class IDriveHardware
{
  public:
    virtual ~IDriveHardware() = default;

    // Commands to control hardware states
    virtual void enableGateDriver() = 0;
    virtual void disableGateDriver() = 0;
    virtual void enablePwm() = 0;
    virtual void disablePwm() = 0;
    virtual void resetControllers() = 0;

    // Queries to check hardware status
    virtual bool isHardwareReady() const = 0;
    virtual bool isGateDriverReady() const = 0;
    virtual bool hasActiveFault() const = 0;
};

/**
 * @brief Event-driven, deterministic Drive State Machine for lifecycle and safety.
 */
class DriveStateMachine
{
  public:
    explicit DriveStateMachine(IDriveHardware& hardware)
        : mHardware(hardware), mState(DriveState::PowerOff)
    {
    }

    /**
     * @brief Transition from PowerOff to Init to begin initialization.
     */
    void initialize()
    {
        if (mState == DriveState::PowerOff)
        {
            transitionTo(DriveState::Init);
        }
    }

    /**
     * @brief Periodic update function. Runs safety checks and handles transient states.
     */
    void update()
    {
        // Fail-safe: transition to Fault if hardware reports active faults
        if (mState != DriveState::PowerOff && mState != DriveState::Fault)
        {
            if (mHardware.hasActiveFault())
            {
                transitionTo(DriveState::Fault);
                return;
            }
        }

        switch (mState)
        {
            case DriveState::PowerOff:
                break;

            case DriveState::Init:
                if (mHardware.isHardwareReady())
                {
                    transitionTo(DriveState::Ready);
                }
                break;

            case DriveState::Ready:
                break;

            case DriveState::Enabling:
                if (mHardware.isGateDriverReady())
                {
                    transitionTo(DriveState::Enabled);
                }
                break;

            case DriveState::Enabled:
                if (!mHardware.isGateDriverReady())
                {
                    transitionTo(DriveState::Fault);
                }
                break;

            case DriveState::Fault:
                break;
        }
    }

    /**
     * @brief Trigger request to start the drive (Ready -> Enabling).
     */
    void startDrive()
    {
        if (mState == DriveState::Ready)
        {
            transitionTo(DriveState::Enabling);
        }
    }

    /**
     * @brief Trigger request to stop the drive (Enabling/Enabled -> Ready).
     */
    void stopDrive()
    {
        if (mState == DriveState::Enabling || mState == DriveState::Enabled)
        {
            transitionTo(DriveState::Ready);
        }
    }

    /**
     * @brief Signal that a fault was detected, forcing an immediate transition to Fault.
     */
    void faultDetected()
    {
        if (mState != DriveState::Fault)
        {
            transitionTo(DriveState::Fault);
        }
    }

    /**
     * @brief Reset fault state (Fault -> Init) if active hardware faults have cleared.
     */
    void resetFault()
    {
        if (mState == DriveState::Fault)
        {
            if (!mHardware.hasActiveFault())
            {
                transitionTo(DriveState::Init);
            }
        }
    }

    /**
     * @brief Get the current drive state.
     */
    DriveState getState() const { return mState; }

    /**
     * @brief Check if the inverter is allowed to produce voltage.
     */
    bool isEnabled() const { return mState == DriveState::Enabled; }

  private:
    void onEnterPowerOff()
    {
        mHardware.disablePwm();
        mHardware.disableGateDriver();
    }

    void onEnterInit()
    {
        mHardware.disablePwm();
        mHardware.disableGateDriver();
    }

    void onEnterReady()
    {
        mHardware.disablePwm();
        mHardware.disableGateDriver();
    }

    void onEnterEnabling()
    {
        mHardware.enableGateDriver();
    }

    void onEnterEnabled()
    {
        mHardware.resetControllers();
        mHardware.enablePwm();
    }

    void onEnterFault()
    {
        mHardware.disablePwm();
        mHardware.disableGateDriver();
    }

    void onExitEnabling(DriveState nextState)
    {
        if (nextState != DriveState::Enabled)
        {
            mHardware.disablePwm();
            mHardware.disableGateDriver();
        }
    }

    void onExitEnabled(DriveState nextState)
    {
        if (nextState != DriveState::Enabling)
        {
            mHardware.disablePwm();
            mHardware.disableGateDriver();
        }
    }

    void transitionTo(DriveState newState)
    {
        if (mState == newState)
        {
            return;
        }

        switch (mState)
        {
            case DriveState::Enabling:
                onExitEnabling(newState);
                break;
            case DriveState::Enabled:
                onExitEnabled(newState);
                break;
            default:
                break;
        }

        mState = newState;

        switch (mState)
        {
            case DriveState::PowerOff:
                onEnterPowerOff();
                break;
            case DriveState::Init:
                onEnterInit();
                break;
            case DriveState::Ready:
                onEnterReady();
                break;
            case DriveState::Enabling:
                onEnterEnabling();
                break;
            case DriveState::Enabled:
                onEnterEnabled();
                break;
            case DriveState::Fault:
                onEnterFault();
                break;
        }
    }

    IDriveHardware& mHardware;
    DriveState mState;
};

} // namespace app
