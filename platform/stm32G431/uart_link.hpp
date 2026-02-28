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
    g_cmd.enable = enable;
    g_cmd.seq++;
}

inline void cmd_set_target_speed(int32_t rpm)
{
    g_cmd.target_speed_rpm = rpm;
    g_cmd.seq++;
}

inline void cmd_set_max_current(int32_t maxCurrent_mA)
{
    g_cmd.max_current_mA = maxCurrent_mA;
    g_cmd.seq++;
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
inline char g_line[96];
inline uint32_t g_line_len = 0;
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
    g_line_ready = 0;

    char* cmd = std::strtok(g_line, " \r\n");
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
    else if (std::strcmp(cmd, "help") == 0)
    {
        const char msg[] = "cmd: en 0|1, spd <rpm>, imax <mA>\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 20);
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