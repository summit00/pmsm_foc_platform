#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C"
{
#include "usart.h"
}

namespace platform
{

// ======================================================
// ===================== COMMAND STATE ==================
// ======================================================

struct ControlInputs
{
    volatile uint8_t enable = 0;
    volatile int32_t target_speed_rpm = 0;
    volatile int32_t max_current_mA = 0;
};

inline ControlInputs g_cmd;

// ======================================================
// ===================== TELEMETRY ======================
// ======================================================

#pragma pack(push, 1)
struct TelemetryPacket
{
    uint16_t sync; // 0xA55A
    uint16_t counter;
    int16_t v[5];
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 14, "TelemetryPacket size mismatch");

inline volatile TelemetryPacket g_telem = {.sync = 0xA55A, .counter = 0, .v = {0, 0, 0, 0, 0}};

inline void telemetry_snapshot_into(TelemetryPacket& dst)
{
    dst.sync = g_telem.sync;
    dst.counter = g_telem.counter;
    for (size_t i = 0; i < 5; i++)
    {
        dst.v[i] = g_telem.v[i];
    }
}

// ======================================================
// ===================== TX (DMA) =======================
// ======================================================

inline volatile uint8_t g_tx_busy = 0;
inline TelemetryPacket g_tx_copy;

inline void telemetry_try_send()
{
    if (g_tx_busy)
        return;

    telemetry_snapshot_into(g_tx_copy); // ensure atomic read of telemetry data
    g_tx_busy = 1;

    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&g_tx_copy, sizeof(g_tx_copy));
}

inline void on_uart_tx_done()
{
    g_tx_busy = 0;
}

// ======================================================
// ===================== RX (INT 1 BYTE) =================
// ======================================================

inline volatile uint8_t g_rx_byte = 0;

inline char g_line[96];
inline uint32_t g_line_len = 0;
inline volatile uint8_t g_line_ready = 0;

inline void uart_init()
{
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&g_rx_byte, 1);
}

inline void on_uart_rx_byte(uint8_t b)
{
    if (g_line_ready)
        return;

    if (b == '\n')
    {
        if (g_line_len < sizeof(g_line))
            g_line[g_line_len] = '\0';

        g_line_len = 0;
        g_line_ready = 1;
        return;
    }

    if (b == '\r')
        return;

    if (g_line_len + 1 < sizeof(g_line))
    {
        g_line[g_line_len++] = static_cast<char>(b);
    }
    else
    {
        g_line_len = 0; // overflow reset
    }
}

inline void process_line()
{
    if (!g_line_ready)
        return;
    g_line_ready = 0;

    char* cmd = std::strtok(g_line, " \r\n");
    if (!cmd)
        return;

    if (std::strcmp(cmd, "en") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            g_cmd.enable = static_cast<uint8_t>(std::atoi(v));
    }
    else if (std::strcmp(cmd, "spd") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            g_cmd.target_speed_rpm = std::atoi(v);
    }
    else if (std::strcmp(cmd, "imax") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            g_cmd.max_current_mA = std::atoi(v);
    }
    else if (std::strcmp(cmd, "help") == 0)
    {
        const char msg[] = "cmd: en 0|1, spd <rpm>, imax <mA>\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 20);
    }
}

} // namespace platform