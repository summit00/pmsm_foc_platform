#pragma once
#include "user_interface.hpp"

namespace platform
{

extern ::app::UserInterface ui;

struct TelemetryRegistryEntry {
    const char* name;
    uint8_t id;
    const float* value_ptr;
    float scale;
    const char* unit;
    const char* description;
};

static constexpr TelemetryRegistryEntry telemetry_registry[] = {
    {"Udc_V",             1,  &::platform::ui.Udc_V,             100.0f,  "V",   "DC Link Bus Voltage"},
    {"demandSpeed_rpm",   2,  &::platform::ui.demandSpeed_rpm,   1.0f,    "rpm", "Demand Electrical Speed"},
    {"feedbackSpeed_rpm", 3,  &::platform::ui.feedbackSpeed_rpm, 1.0f,    "rpm", "Feedback Rotor Speed"},
    {"encoderSpeed_rpm",  4,  &::platform::ui.encoderSpeed_rpm,  1.0f,    "rpm", "Encoder Measured Speed"},
    {"observerSpeed_rpm", 5,  &::platform::ui.observerSpeed_rpm, 1.0f,    "rpm", "Observer Estimated Speed"},
    {"openLoopSpeed_rpm", 6,  &::platform::ui.openLoopSpeed_rpm, 1.0f,    "rpm", "Open Loop Speed"},
    {"Id_A",              7,  &::platform::ui.Id_A,              1000.0f, "A",   "D-axis feedback current"},
    {"Iq_A",              8,  &::platform::ui.Iq_A,              1000.0f, "A",   "Q-axis feedback current"},
    {"IdRef_A",           9,  &::platform::ui.IdRef_A,           1000.0f, "A",   "D-axis reference current"},
    {"IqRef_A",           10, &::platform::ui.IqRef_A,           1000.0f, "A",   "Q-axis reference current"},
    {"Ud_V",              11, &::platform::ui.Ud_V,              1000.0f, "V",   "D-axis voltage command"},
    {"Uq_V",              12, &::platform::ui.Uq_V,              1000.0f, "V",   "Q-axis voltage command"},
    {"encoderAngle_deg",  13, &::platform::ui.encoderAngle_deg,  100.0f,  "deg", "Rotor electrical angle from encoder"},
    {"observerAngle_deg", 14, &::platform::ui.observerAngle_deg, 100.0f,  "deg", "Rotor electrical angle from observer"},
    {"angleError_deg",    15, &::platform::ui.angleError_deg,    100.0f,  "deg", "Error between encoder and observer angle"},
    {"Ialpha_A",          16, &::platform::ui.Ialpha_A,          1000.0f, "A",   "Alpha current component"},
    {"Ibeta_A",           17, &::platform::ui.Ibeta_A,           1000.0f, "A",   "Beta current component"},
    {"Ualpha_V",          18, &::platform::ui.Ualpha_V,          1000.0f, "V",   "Alpha voltage component"},
    {"Ubeta_V",           19, &::platform::ui.Ubeta_V,           1000.0f, "V",   "Beta voltage component"},
    {"temp_C",            20, &::platform::ui.temp_C,            10.0f,   "C",   "Inverter temperature"},
    {"errorState",        21, &::platform::ui.errorState,        1.0f,    "",    "Motor error/fault state code"},
    {"autoSetupState",    22, &::platform::ui.autoSetupState,    1.0f,    "",    "Autosetup sequence state code"}
};

static constexpr size_t TELEMETRY_REGISTRY_SIZE = sizeof(telemetry_registry) / sizeof(telemetry_registry[0]);

} // namespace platform
