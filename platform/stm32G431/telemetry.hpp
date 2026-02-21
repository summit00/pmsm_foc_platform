#pragma once
#include "interfaces.hpp"
#include "uart_link.hpp"

namespace platform
{
class Telemetry : public app::ITelemetry
{
  public:
    void publish5_i16(int16_t v0, int16_t v1, int16_t v2, int16_t v3, int16_t v4) override
    {
        g_telem.v[0] = v0;
        g_telem.v[1] = v1;
        g_telem.v[2] = v2;
        g_telem.v[3] = v3;
        g_telem.v[4] = v4;
    }
};
} // namespace platform