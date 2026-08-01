#pragma once
#include <cstring>
#include "hal/itransfer.hpp"
#include "comm/telemetry_manager.hpp"

#ifndef __disable_irq
#define __disable_irq()
#define __enable_irq()
#endif

namespace comm
{

class ProtocolHandler
{
  public:
    struct RxCommand
    {
        int32_t enable;
        int32_t mode;
        float targetSpeed_rpm;
        float accel_rpm_s;
        float isAbs_mA;
    };

    using RxCallback = void(*)(const RxCommand&, void* ctx);

    ProtocolHandler(hal::ITransfer& transfer, TelemetryManager& telemetry)
        : transfer_(transfer), telemetry_(telemetry)
    {
        transfer_.set_rx_callback(on_raw_rx, this);
    }

    void setRxCallback(RxCallback cb, void* ctx)
    {
        rxCb_ = cb;
        rxCtx_ = ctx;
    }

    // Call from background loop at ~1 ms
    void update()
    {
        send_telemetry();
    }

  private:
    static constexpr uint16_t RX_MAGIC = 0xABCDu;
    static constexpr uint16_t SELECT_MAGIC = 0xABCEu;
    static constexpr uint16_t TX_MAGIC = 0xDCBAu;
    static constexpr uint16_t PAYLOAD_N = 10u;
    static constexpr uint16_t FRAME_BYTES = 4u + PAYLOAD_N * 4u; // 44 bytes

    static constexpr size_t MAX_BATCH_SAMPLES = 240;
    static constexpr size_t TX_BATCH_HEADER_BYTES = 6;
    static constexpr size_t TX_BUFFER_BYTES =
        TX_BATCH_HEADER_BYTES + MAX_BATCH_SAMPLES * sizeof(platform::Sample);

    hal::ITransfer& transfer_;
    TelemetryManager& telemetry_;

    uint16_t tx_seq_ = 0;
    uint16_t last_rx_seq_ = 0xFFFFu;

    uint8_t tx_buf_[TX_BUFFER_BYTES] = {};

    RxCallback rxCb_ = nullptr;
    void* rxCtx_ = nullptr;

    // Encodes telemetry queue into a packet frame and sends via transfer layer
    void send_telemetry()
    {
        if (transfer_.is_tx_busy())
        {
            return; // Busy transmitting previous batch
        }

        size_t avail = telemetry_.available_samples();
        if (avail == 0)
        {
            return;
        }

        size_t count = avail;
        if (count > MAX_BATCH_SAMPLES)
        {
            count = MAX_BATCH_SAMPLES;
        }

        // Fill header
        uint16_t magic = TX_MAGIC;
        uint16_t count_u16 = static_cast<uint16_t>(count);
        std::memcpy(&tx_buf_[0], &magic, 2);
        std::memcpy(&tx_buf_[2], &tx_seq_, 2);
        std::memcpy(&tx_buf_[4], &count_u16, 2);

        // Pop samples into buffer
        uint8_t* ptr = &tx_buf_[TX_BATCH_HEADER_BYTES];
        for (size_t i = 0; i < count; ++i)
        {
            platform::Sample s;
            if (telemetry_.pop_sample(s))
            {
                std::memcpy(ptr, &s, sizeof(platform::Sample));
                ptr += sizeof(platform::Sample);
            }
        }

        uint16_t total_bytes =
            static_cast<uint16_t>(TX_BATCH_HEADER_BYTES + count * sizeof(platform::Sample));

        if (transfer_.send(tx_buf_, total_bytes))
        {
            ++tx_seq_;
        }
    }

    // Static callback matching ITransfer::RxCallback
    static void on_raw_rx(const uint8_t* data, size_t len, void* ctx)
    {
        auto* self = static_cast<ProtocolHandler*>(ctx);
        self->process_rx_bytes(data, len);
    }

    // Decodes bytes, validates sequences, and processes commands or configurations
    void process_rx_bytes(const uint8_t* data, size_t len)
    {
        if (len < FRAME_BYTES)
        {
            return;
        }

        // Validate magic
        uint16_t magic;
        std::memcpy(&magic, &data[0], 2);
        if (magic != RX_MAGIC && magic != SELECT_MAGIC)
        {
            return;
        }

        // Validate sequence number (skip duplicates)
        uint16_t seq;
        std::memcpy(&seq, &data[2], 2);
        if (seq == last_rx_seq_)
        {
            return;
        }
        last_rx_seq_ = seq;

        if (magic == RX_MAGIC)
        {
            // Parse command payload
            int32_t p[PAYLOAD_N];
            std::memcpy(p, &data[4], PAYLOAD_N * sizeof(int32_t));

            __disable_irq();
            RxCommand cmd{p[0],
                          p[1],
                          static_cast<float>(p[2]) * 0.01f,
                          static_cast<float>(p[3]) * 0.01f,
                          static_cast<float>(p[4]) * 0.1f};
            if (rxCb_)
            {
                rxCb_(cmd, rxCtx_);
            }
            __enable_irq();
        }
        else if (magic == SELECT_MAGIC)
        {
            // Parse dynamic telemetry ID selection
            uint16_t ids[10];
            std::memcpy(ids, &data[4], 10 * sizeof(uint16_t));
            telemetry_.set_selected_ids(ids, 10);
        }
    }
};

} // namespace comm
