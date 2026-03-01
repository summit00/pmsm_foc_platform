#pragma once
#include <cstdint>

namespace app
{

struct ITickMs
{
    virtual ~ITickMs() = default;
    virtual uint32_t now_ms() const = 0;
};

struct IDigitalOut
{
    virtual ~IDigitalOut() = default;
    virtual void toggle() = 0;
};

struct PhaseCurrentsRaw
{
    uint16_t ia_counts;
    uint16_t ib_counts;
};

struct PhaseCurrents
{
    float ia_A;
    float ib_A;
};

class ICurrentSense
{
  public:
    virtual ~ICurrentSense() = default;
    virtual PhaseCurrentsRaw read_raw() const = 0;
    virtual PhaseCurrents read_amps() const = 0; // <-- new
    virtual void calibrate_offset() = 0;         // <-- new: call at startup with no current
};

class ITelemetry
{
  public:
    virtual ~ITelemetry() = default;
    virtual void publish5_i16(int16_t v0, int16_t v1, int16_t v2, int16_t v3, int16_t v4) = 0;
};

struct ControlInputs
{
    uint8_t enable = 0;
    int32_t target_speed_rpm = 0;
    int32_t max_current_mA = 0;
    volatile uint32_t seq;
};

class IControlInputs
{
  public:
    virtual ~IControlInputs() = default;
    virtual ControlInputs read() const = 0;
};

struct PhaseVoltages
{
    float va_V;
    float vb_V;
    float vc_V;
};

class IInverter
{
  public:
    virtual ~IInverter() = default;
    virtual void set_phase_voltages(float va_V, float vb_V, float vc_V, float vbus_V) = 0;
};

class IEnableOutput
{
  public:
    virtual ~IEnableOutput() = default;
    virtual void set_enable(bool enabled) = 0;
};

class ICycleCounter
{
  public:
    virtual ~ICycleCounter() = default;
    virtual uint32_t now_cycles() const = 0;
    virtual uint32_t cycles_per_second() const = 0;
};

} // namespace app
