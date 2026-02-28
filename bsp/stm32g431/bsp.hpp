#pragma once

#include "pinning.hpp"

namespace bsp
{

// ======================
// Board-Level Aliases
// ======================

// Nucleo-style status LED = PA5
inline constexpr pinning::GpioPin status_led()
{
    return pinning::PA5;
}

inline constexpr pinning::GpioPin powerstage_enable_a()
{
    return pinning::PC10;
}

inline constexpr pinning::GpioPin powerstage_enable_b()
{
    return pinning::PC11;
}
inline constexpr pinning::GpioPin powerstage_enable_c()
{
    return pinning::PC12;
}

} // namespace bsp
