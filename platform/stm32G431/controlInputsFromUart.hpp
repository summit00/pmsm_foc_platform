#pragma once
#include <cstdint>

#include "interfaces.hpp"
#include "uart_link.hpp"

namespace platform
{

// inline volatile uint32_t g_cmd_seq = 0;

// inline void cmd_set_enable(uint8_t enable)
// {
//     g_cmd.enable = enable;
//     g_cmd_seq++;
// }

// inline void cmd_set_target_speed(int32_t rpm)
// {
//     g_cmd.target_speed_rpm = rpm;
//     g_cmd_seq++;
// }

// inline void cmd_set_max_current(int32_t maxCurrent_mA)
// {
//     g_cmd.max_current_mA = maxCurrent_mA;
//     g_cmd_seq++;
// }

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
            out.enable = platform::g_cmd.enable;
            out.target_speed_rpm = platform::g_cmd.target_speed_rpm;
            out.max_current_mA = platform::g_cmd.max_current_mA;
            out.seq = s1;
            s2 = platform::g_cmd.seq;
        } while (s1 != s2); // ensure we read a consistent snapshot
        return out;
    }
};

} // namespace platform