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
    {"autoSetupState",    22, &::platform::ui.autoSetupState,    1.0f,    "",    "Autosetup sequence state code"},
    {"drvFault1",         23, &::platform::ui.drvFault1,         1.0f,    "",    "DRV8353 Fault Status 1 Register"},
    {"drvFault2",         24, &::platform::ui.drvFault2,         1.0f,    "",    "DRV8353 Fault Status 2 Register"},
    {"drvInitOk",         25, &::platform::ui.drvInitOk,         1.0f,    "",    "DRV8353 SPI Init OK (1=Yes, 0=No)"},
    {"cmdEnable",         26, &::platform::ui.cmdEnable,         1.0f,    "",    "Commanded Motor Enable state"},
    {"cmdMode",           27, &::platform::ui.cmdMode,           1.0f,    "",    "Commanded Mode state"},
    {"rxPackets",         28, &::platform::ui.rxPackets,         1.0f,    "",    "USB Command Packets Received"},
    {"driveState",        29, &::platform::ui.driveState,        1.0f,    "",    "Drive State (0=PwrOff,1=Init,2=Ready,3=Enabling,4=Enabled,5=Fault)"},
    {"Rs_ohm",            30, &::platform::ui.Rs_ohm,            1000.0f, "Ohm", "Identified stator phase resistance"},
    {"Ld_uH",             31, &::platform::ui.Ld_uH,             1.0f,    "uH",  "Direct-axis inductance"},
    {"Lq_uH",             32, &::platform::ui.Lq_uH,             1.0f,    "uH",  "Quadrature-axis inductance"},
    {"flux_pm_mWb",       33, &::platform::ui.flux_pm_mWb,       100.0f,  "mWb", "PM Flux Linkage"},
    {"encoderOffset",     34, &::platform::ui.encoderOffset_ticks, 1.0f,  "ticks", "Calibrated encoder electrical offset"},
    {"encoderIndex",      35, &::platform::ui.encoderIndexFound,  1.0f,    "",    "Encoder Index Z-pulse captured (1=Yes, 0=No)"},
    {"encoderTicks",      36, &::platform::ui.encoderRawTicks,    1.0f,    "ticks", "Encoder raw timer ticks"},
    {"runtimeTicks",      37, &::platform::ui.runtimeTicks,       1.0f,    "ticks", "FOC ISR execution runtime in CPU cycles/ticks"},
    {"runtime_us",        38, &::platform::ui.runtime_us,         10.0f,   "us",   "FOC ISR execution time in microseconds"}
};

static constexpr size_t TELEMETRY_REGISTRY_SIZE = sizeof(telemetry_registry) / sizeof(telemetry_registry[0]);

} // namespace platform
