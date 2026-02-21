#pragma once
#include <cstdint>

#pragma pack(push, 1)
struct TelemetryPacket
{
    uint16_t sync;
    uint16_t counter;
    int16_t v[5];
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 2 + 2 + 10, "TelemetryPacket size mismatch");