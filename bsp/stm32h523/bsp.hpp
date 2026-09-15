#pragma once

#include "pinning.hpp"
#include "powerstage_parameters.hpp"

namespace bsp
{

// ======================
// Board-Level Aliases
// ======================

// Status LED = PD3
inline constexpr pinning::GpioPin status_led()
{
    return pinning::PD3;
}

// Error LED = PD5
inline constexpr pinning::GpioPin error_led()
{
    return pinning::PD5;
}

// Gate Driver Enable = PB10
inline constexpr pinning::GpioPin drv_enable()
{
    return pinning::PB10;
}

// Gate Driver Fault = PB12
inline constexpr pinning::GpioPin drv_fault()
{
    return pinning::PB12;
}

// SPI Chip Select = PB6
inline constexpr pinning::GpioPin chip_select()
{
    return pinning::PB6;
}

// USB Vbus = PA10
inline constexpr pinning::GpioPin usb_vbus()
{
    return pinning::PA10;
}

} // namespace bsp
