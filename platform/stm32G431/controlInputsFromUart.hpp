#pragma once
#include <cstdint>

#include "interfaces.hpp"
#include "uart_link.hpp"

namespace platform
{

class ControlInputsFromUart final : public app::IControlInputs
{
  public:
    app::ControlInputs read() const override
    {
        app::ControlInputs out{};
        uint32_t s1, s2;

        do
        {
            s1 = platform::g_cmd.seq;
            if (s1 & 1)
                continue; // write in progress, spin
            __DMB();
            out.enable = platform::g_cmd.enable;
            out.target_speed_rpm = platform::g_cmd.target_speed_rpm;
            out.max_current_mA = platform::g_cmd.max_current_mA;
            out.seq = s1;
            __DMB();
            s2 = platform::g_cmd.seq;
        } while (s1 != s2);

        return out;
    }
};

} // namespace platform