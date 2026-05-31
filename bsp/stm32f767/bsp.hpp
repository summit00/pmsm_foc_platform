#pragma once

#include "pinning.hpp"

namespace bsp
{

// ======================
// Board-Level Aliases
// ======================

// Nucleo-style status LED = PA5
// inline constexpr pinning::GpioPin status_led()
// {
//     return pinning::PA5;
// }

inline constexpr pinning::GpioPin powerstage_enable_a()
{
    return pinning::PD5;
}

inline constexpr pinning::GpioPin powerstage_enable_b()
{
    return pinning::PD6;
}
inline constexpr pinning::GpioPin powerstage_enable_c()
{
    return pinning::PD7;
}

inline constexpr pinning::GpioPin powerstage_enable_general()
{
    return pinning::PD4;
}

} // namespace bsp
