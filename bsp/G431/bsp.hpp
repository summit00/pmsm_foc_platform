#pragma once

#include "pinning.hpp"

namespace bsp {

// ======================
// Board-Level Aliases
// ======================

// Nucleo-style status LED = PA5
inline constexpr pinning::GpioPin status_led()
{
    return pinning::PA5;
}

} // namespace bsp
