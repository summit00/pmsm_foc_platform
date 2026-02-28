#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "interfaces.hpp"

extern "C"
{
#include "usart.h"
}

namespace platform
{

// ======================================================
// ===================== COMMAND STATE ==================
// ======================================================
inline volatile app::ControlInputs g_cmd = {0, 0, 0, 0};

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

inline void cmd_set_enable(uint8_t enable)
{
    g_cmd.seq++; // now odd → writer active
    __DMB();     // memory barrier: field write must come after
    g_cmd.enable = enable;
    __DMB();     // barrier: seq++ must come after field write
    g_cmd.seq++; // now even → stable
}

inline void cmd_set_target_speed(int32_t rpm)
{
    g_cmd.seq++;
    __DMB();
    g_cmd.target_speed_rpm = rpm;
    __DMB();
    g_cmd.seq++;
}

inline void cmd_set_max_current(int32_t maxCurrent_mA)
{
    g_cmd.seq++;
    __DMB();
    g_cmd.max_current_mA = maxCurrent_mA;
    __DMB();
    g_cmd.seq++;
}

// ======================================================
// ===================== TX (DMA) =======================
// ======================================================

inline volatile bool g_tx_busy = false;
inline TelemetryPacket g_tx_copy;

inline void telemetry_try_send()
{
    if (g_tx_busy)
        return;

    g_tx_busy = true;
    telemetry_snapshot_into(g_tx_copy);
    g_tx_copy.counter++;

    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&g_tx_copy, sizeof(g_tx_copy));
}

inline void on_uart_tx_done()
{
    g_tx_busy = false;
}

// ======================================================
// ===================== RX (INT 1 BYTE) =================
// ======================================================
inline volatile char g_line[96];
inline volatile uint32_t g_line_len = 0;
inline volatile uint8_t g_line_ready = 0;

#define RX_DMA_BUF_SIZE 256
inline uint8_t g_rx_dma_buf[RX_DMA_BUF_SIZE];
inline uint32_t g_rd_ptr = 0; // Tracks processed data position

inline void uart_init()
{
    // Start DMA in circular mode once
    HAL_UART_Receive_DMA(&huart2, g_rx_dma_buf, RX_DMA_BUF_SIZE);
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

    // Copy first, then release the buffer
    char local[sizeof(g_line)];
    memcpy(local, (const char*)g_line, sizeof(g_line));
    g_line_len = 0;
    g_line_ready = 0; // release AFTER copy

    char* cmd = std::strtok(local, " \r\n");
    if (!cmd)
        return;

    if (std::strcmp(cmd, "en") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            cmd_set_enable(static_cast<uint8_t>(std::atoi(v)));
    }
    else if (std::strcmp(cmd, "spd") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            cmd_set_target_speed(std::atoi(v));
    }
    else if (std::strcmp(cmd, "imax") == 0)
    {
        char* v = std::strtok(nullptr, " \r\n");
        if (v)
            cmd_set_max_current(std::atoi(v));
    }
}

inline void check_for_rx_data()
{
    // Calculate current write position from DMA CNDTR register
    // CNDTR counts DOWN from RX_DMA_BUF_SIZE to 0
    uint32_t curr_wr_ptr = RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    while (g_rd_ptr != curr_wr_ptr)
    {
        uint8_t b = g_rx_dma_buf[g_rd_ptr];

        // Use your existing logic to parse lines
        on_uart_rx_byte(b);

        g_rd_ptr = (g_rd_ptr + 1) % RX_DMA_BUF_SIZE;
    }
}

} // namespace platform