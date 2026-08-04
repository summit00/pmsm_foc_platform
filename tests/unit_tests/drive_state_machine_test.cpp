#include "drive_state_machine.hpp"
#include <catch2/catch_test_macros.hpp>

using namespace app;

class MockDriveHardware : public IDriveHardware
{
  public:
    void enableGateDriver() override { gateDriverEnabled = true; }
    void disableGateDriver() override { gateDriverEnabled = false; }
    void enablePwm() override { pwmEnabled = true; }
    void disablePwm() override { pwmEnabled = false; }
    void resetControllers() override { controllersResetCalled = true; }

    bool isHardwareReady() const override { return hardwareReady; }
    bool isGateDriverReady() const override { return gateDriverReady; }
    bool hasActiveFault() const override { return activeFault; }

    bool gateDriverEnabled = false;
    bool pwmEnabled = false;
    bool controllersResetCalled = false;

    bool hardwareReady = false;
    bool gateDriverReady = false;
    bool activeFault = false;
};

TEST_CASE("DriveStateMachine Lifecycle and Safety Test")
{
    MockDriveHardware hw;
    DriveStateMachine dsm(hw);

    SECTION("Initial state is PowerOff and outputs are disabled")
    {
        REQUIRE(dsm.getState() == DriveState::PowerOff);
        REQUIRE_FALSE(dsm.isEnabled());
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Initialization sequence")
    {
        dsm.initialize();
        REQUIRE(dsm.getState() == DriveState::Init);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);

        // Hardware not ready yet, update should stay in Init
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Init);

        // Hardware ready, update should transition to Ready
        hw.hardwareReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Ready);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Ready to Enabled transition sequence")
    {
        // Setup state to Ready
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Ready);

        // Try startDrive
        dsm.startDrive();
        REQUIRE(dsm.getState() == DriveState::Enabling);
        REQUIRE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);

        // Wait for gate driver to be ready
        dsm.update(); // gate driver not ready yet
        REQUIRE(dsm.getState() == DriveState::Enabling);
        REQUIRE_FALSE(hw.pwmEnabled);

        hw.gateDriverReady = true;
        dsm.update(); // gate driver ready now
        REQUIRE(dsm.getState() == DriveState::Enabled);
        REQUIRE(dsm.isEnabled());
        REQUIRE(hw.gateDriverEnabled);
        REQUIRE(hw.controllersResetCalled);
        REQUIRE(hw.pwmEnabled);
    }

    SECTION("Stop drive from Enabled state")
    {
        // Setup state to Enabled
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        hw.gateDriverReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Enabled);

        // Stop drive
        dsm.stopDrive();
        REQUIRE(dsm.getState() == DriveState::Ready);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Stop drive from Enabling state")
    {
        // Setup state to Enabling
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        REQUIRE(dsm.getState() == DriveState::Enabling);

        // Stop drive
        dsm.stopDrive();
        REQUIRE(dsm.getState() == DriveState::Ready);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("External fault detection disables power stage immediately")
    {
        // Setup state to Enabled
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        hw.gateDriverReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Enabled);

        // Trigger fault
        dsm.faultDetected();
        REQUIRE(dsm.getState() == DriveState::Fault);
        REQUIRE_FALSE(dsm.isEnabled());
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Active fault in update transitions to Fault automatically")
    {
        // Setup state to Enabled
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        hw.gateDriverReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Enabled);

        // Trigger fault at hardware level
        hw.activeFault = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Fault);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Gate driver loss during Enabled transitions to Fault")
    {
        // Setup state to Enabled
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        hw.gateDriverReady = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Enabled);

        // Gate driver becomes unready
        hw.gateDriverReady = false;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Fault);
        REQUIRE_FALSE(hw.gateDriverEnabled);
        REQUIRE_FALSE(hw.pwmEnabled);
    }

    SECTION("Fault reset behavior depends on fault clearance")
    {
        dsm.initialize();
        hw.hardwareReady = true;
        dsm.update();
        dsm.startDrive();
        hw.gateDriverReady = true;
        dsm.update();
        
        hw.activeFault = true;
        dsm.update();
        REQUIRE(dsm.getState() == DriveState::Fault);

        // Attempt reset while fault is still active
        dsm.resetFault();
        REQUIRE(dsm.getState() == DriveState::Fault); // remains in Fault

        // Clear fault and reset
        hw.activeFault = false;
        dsm.resetFault();
        REQUIRE(dsm.getState() == DriveState::Init);
    }
}
