#include "fault_manager.hpp"
#include <catch2/catch_test_macros.hpp>

using namespace app;

TEST_CASE("FaultManager - Normal operation and threshold safety tests")
{
    FaultThresholds customThresholds{
        .overcurrent_threshold_A = 15.0f,
        .overvoltage_threshold_V = 50.0f,
        .undervoltage_threshold_V = 12.0f,
        .overtemp_threshold_C = 75.0f
    };

    FaultManager fm(customThresholds);

    SECTION("Normal conditions produce no faults")
    {
        fm.checkForFaults(5.0f, -2.5f, 24.0f, 40.0f);
        REQUIRE_FALSE(fm.isFaulted());
        REQUIRE(fm.getFaultType() == static_cast<uint8_t>(FaultManager::FaultType::NONE));
    }

    SECTION("Overcurrent detection on Phase A")
    {
        // Positive overcurrent
        fm.checkForFaults(16.0f, 0.0f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT)) != 0);

        fm.clearFaults();
        REQUIRE_FALSE(fm.isFaulted());

        // Negative overcurrent
        fm.checkForFaults(-15.5f, 0.0f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT)) != 0);
    }

    SECTION("Overcurrent detection on Phase C")
    {
        fm.checkForFaults(0.0f, 15.1f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT)) != 0);

        fm.clearFaults();
        fm.checkForFaults(0.0f, -16.0f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
    }

    SECTION("Overcurrent detection on calculated Phase B (-ia - ic)")
    {
        // ia = 8.0A, ic = 8.0A -> ib = -16.0A which exceeds 15.0A threshold
        fm.checkForFaults(8.0f, 8.0f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT)) != 0);
    }

    SECTION("Overvoltage detection")
    {
        fm.checkForFaults(0.0f, 0.0f, 52.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERVOLTAGE)) != 0);
    }

    SECTION("Undervoltage detection")
    {
        fm.checkForFaults(0.0f, 0.0f, 11.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::UNDERVOLTAGE)) != 0);
    }

    SECTION("Overtemperature detection")
    {
        fm.checkForFaults(0.0f, 0.0f, 24.0f, 80.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERTEMP)) != 0);
    }

    SECTION("Combined multiple faults")
    {
        fm.checkForFaults(20.0f, 0.0f, 55.0f, 90.0f);
        REQUIRE(fm.isFaulted());
        uint8_t expected = static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT) |
                           static_cast<uint8_t>(FaultManager::FaultType::OVERVOLTAGE) |
                           static_cast<uint8_t>(FaultManager::FaultType::OVERTEMP);
        REQUIRE(fm.getFaultType() == expected);
    }

    SECTION("Updating thresholds dynamically")
    {
        FaultThresholds tightened{
            .overcurrent_threshold_A = 3.0f,
            .overvoltage_threshold_V = 30.0f,
            .undervoltage_threshold_V = 20.0f,
            .overtemp_threshold_C = 50.0f
        };
        fm.setThresholds(tightened);

        // 4.0A was safe under 15A, but trips under 3A
        fm.checkForFaults(4.0f, 0.0f, 24.0f, 40.0f);
        REQUIRE(fm.isFaulted());
        REQUIRE((fm.getFaultType() & static_cast<uint8_t>(FaultManager::FaultType::OVERCURRENT)) != 0);
    }
}
