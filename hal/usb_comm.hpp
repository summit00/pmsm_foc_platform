#pragma once
#include <cstdint>
#include <cstring>
#include "user_interface.hpp"

extern "C"
{
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
}

// ---------------------------------------------------------------------------
// Wire protocol — fixed-size binary frames, int32_t payload.
//
// Sending floats as scaled int32_t avoids any float/endian issues across
// the PC↔MCU boundary.
//
// PC → MCU  (RX_FRAME_SIZE = 44 bytes)
//   [0..1]  uint16_t  magic  = 0xABCD
//   [2..3]  uint16_t  seq    (PC-side sequence counter, ignored by MCU)
//   [4..43] int32_t[10]:
//     [0]  enable          (0 = off, 1 = on)
//     [1]  mode            (0 = openloop, 1 = closedloop, 2 = autosetup)
//     [2]  targetSpeed_rpm × 100   (e.g. 3000 rpm → 300000)
//     [3]  accel_rpm_s     × 100
//     [4]  isAbs_mA        × 10
//     [5..9] reserved, send 0
//
// MCU → PC  (TX_FRAME_SIZE = 44 bytes)
//   [0..1]  uint16_t  magic  = 0xDCBA
//   [2..3]  uint16_t  seq    (MCU increments each TX)
//   [4..43] int32_t[10]:
//     [0]  actualSpeed_rpm         × 100
//     [1]  busVoltage_V            × 1000
//     [2]  Id_A                    × 1000
//     [3]  Iq_A                    × 1000
//     [4]  IdRef_A                 × 1000
//     [5]  IqRef_A                 × 1000
//     [6]  ThetaEncoder_deg        × 100
//     [7]  ThetaOpenLoop_deg       × 100
//     [8]  actualSpeedEncoder_rpm  × 100
//     [9]  Udc_V                   × 1000
// ---------------------------------------------------------------------------

namespace platform
{

extern ::app::UserInterface ui;

struct RxCommand {
    int32_t enable;
    int32_t mode;
    float targetSpeed_rpm;
    float accel_rpm_s;
    float isAbs_mA;
};

static constexpr uint16_t USB_RX_MAGIC = 0xABCDu;
static constexpr uint16_t USB_SELECT_MAGIC = 0xABCEu;
static constexpr uint16_t USB_TX_MAGIC = 0xDCBAu;
static constexpr uint16_t USB_PAYLOAD_N = 10u;
static constexpr uint16_t USB_FRAME_BYTES = 4u + USB_PAYLOAD_N * 4u; // 44 for RX commands

struct __attribute__((packed)) Sample {
    uint8_t id;
    int16_t value;
};

struct TelemetryRegistryEntry {
    const char* name;
    uint8_t id;
    const float* value_ptr;
    float scale;
};

static constexpr TelemetryRegistryEntry telemetry_registry[] = {
    {"Udc_V",             1,  &ui.Udc_V,             100.0f},
    {"demandSpeed_rpm",   2,  &ui.demandSpeed_rpm,   1.0f},
    {"feedbackSpeed_rpm", 3,  &ui.feedbackSpeed_rpm, 1.0f},
    {"encoderSpeed_rpm",  4,  &ui.encoderSpeed_rpm,  1.0f},
    {"observerSpeed_rpm", 5,  &ui.observerSpeed_rpm, 1.0f},
    {"Id_A",              6,  &ui.Id_A,              1000.0f},
    {"Iq_A",              7,  &ui.Iq_A,              1000.0f},
    {"encoderAngle_deg",  8,  &ui.encoderAngle_deg,  100.0f},
    {"observerAngle_deg", 9,  &ui.observerAngle_deg, 100.0f},
    {"angleError_deg",    10, &ui.angleError_deg,    100.0f}
};

// Lock-free single-producer, single-consumer ring buffer
template <typename T, size_t Size> class RingBuffer
{
  public:
    bool push(const T& item)
    {
        size_t next = (head_ + 1) % Size;
        if (next == tail_)
        {
            return false; // Overflow
        }
        data_[head_] = item;
        head_ = next;
        return true;
    }

    bool pop(T& item)
    {
        if (head_ == tail_)
        {
            return false; // Underflow
        }
        item = data_[tail_];
        tail_ = (tail_ + 1) % Size;
        return true;
    }

    size_t size() const
    {
        size_t h = head_;
        size_t t = tail_;
        if (h >= t)
        {
            return h - t;
        }
        return Size + h - t;
    }

  private:
    volatile size_t head_ = 0;
    volatile size_t tail_ = 0;
    T data_[Size];
};

// External symbols from usbd_cdc_if.c — not modified, just referenced.
extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" USBD_HandleTypeDef hUsbDeviceFS;

// ---------------------------------------------------------------------------
class UsbComm
{
  public:
    using RxCallback = void(*)(const RxCommand&, void* ctx);
    void setRxCallback(RxCallback cb, void* ctx) { rxCb_ = cb; rxCtx_ = ctx; }

    uint16_t selectedIds_[10] = {};

    void setSelectedIds(const uint16_t* ids, size_t n)
    {
        __disable_irq();
        size_t count = (n > 10) ? 10 : n;
        for (size_t i = 0; i < count; ++i)
        {
            selectedIds_[i] = ids[i];
        }
        for (size_t i = count; i < 10; ++i)
        {
            selectedIds_[i] = 0;
        }
        __enable_irq();
    }

    // Call once after MX_USB_DEVICE_Init().
    void init()
    {
        tx_seq_ = 0;
        last_rx_seq_ = 0xFFFFu; // force first frame to be accepted
        for (size_t i = 0; i < 10; ++i)
        {
            selectedIds_[i] = telemetry_registry[i].id;
        }
    }

    // Call from the main loop at ~1 ms.
    // 1. Poll for a newly received RX frame → apply commands via callback.
    // 2. Snapshot telemetry from ring buffer → transmit TX frame.
    void update()
    {
        poll_rx();
        send_telemetry();
    }

    // Call from ISR to push a sample to the queue
    void push_sample(const Sample& sample)
    {
        tx_queue_.push(sample);
    }

  private:
    uint16_t tx_seq_ = 0;
    uint16_t last_rx_seq_ = 0;

    static constexpr size_t MAX_BATCH_SAMPLES = 240;
    static constexpr size_t TX_BATCH_HEADER_BYTES = 6;
    static constexpr size_t TX_BUFFER_BYTES =
        TX_BATCH_HEADER_BYTES + MAX_BATCH_SAMPLES * sizeof(Sample);

    uint8_t tx_buf_[TX_BUFFER_BYTES] = {};
    RingBuffer<Sample, 2048> tx_queue_;

    RxCallback rxCb_ = nullptr;
    void* rxCtx_ = nullptr;

    // -----------------------------------------------------------------------
    // RX — the USB CDC stack writes received bytes directly into UserRxBufferFS
    // and calls CDC_Receive_FS (static in usbd_cdc_if.c, not patchable).
    void poll_rx()
    {
        if (hUsbDeviceFS.pClassData == nullptr)
            return;

        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);

        uint16_t rx_len = static_cast<uint16_t>(hcdc->RxLength);
        if (rx_len < USB_FRAME_BYTES)
            return;

        // Snapshot under __disable_irq to prevent USB IRQ from overwriting
        // UserRxBufferFS while we're reading it.
        uint8_t local[USB_FRAME_BYTES];
        __disable_irq();
        memcpy(local, UserRxBufferFS, USB_FRAME_BYTES);
        __enable_irq();

        // Validate magic.
        uint16_t magic;
        memcpy(&magic, &local[0], 2);
        if (magic != USB_RX_MAGIC && magic != USB_SELECT_MAGIC)
            return;

        // Deduplicate by sequence number: skip frames we already processed.
        uint16_t seq;
        memcpy(&seq, &local[2], 2);
        if (seq == last_rx_seq_)
            return;
        last_rx_seq_ = seq;

        if (magic == USB_RX_MAGIC)
        {
            // Parse payload (little-endian int32_t array).
            int32_t p[USB_PAYLOAD_N];
            memcpy(p, &local[4], USB_PAYLOAD_N * sizeof(int32_t));

            // Apply commands — written from main loop, read by ADC IRQ.
            __disable_irq();
            RxCommand cmd{p[0], p[1], static_cast<float>(p[2]) * 0.01f,
                          static_cast<float>(p[3]) * 0.01f, static_cast<float>(p[4]) * 0.1f};
            if (rxCb_) rxCb_(cmd, rxCtx_);
            __enable_irq();
        }
        else if (magic == USB_SELECT_MAGIC)
        {
            // Parse payload as up to 10 uint16_t ids.
            uint16_t ids[10];
            memcpy(ids, &local[4], 10 * sizeof(uint16_t));
            setSelectedIds(ids, 10);
        }
    }

    // -----------------------------------------------------------------------
    // TX — snapshot telemetry and send a binary frame.
    void send_telemetry()
    {
        if (hUsbDeviceFS.pClassData == nullptr)
            return;

        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
        if (hcdc->TxState != 0)
        {
            return; // USB is busy transmitting the previous batch, try next frame
        }

        size_t avail = tx_queue_.size();
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
        uint16_t magic = USB_TX_MAGIC;
        uint16_t count_u16 = static_cast<uint16_t>(count);
        memcpy(&tx_buf_[0], &magic, 2);
        memcpy(&tx_buf_[2], &tx_seq_, 2);
        memcpy(&tx_buf_[4], &count_u16, 2);

        // Pop samples into buffer
        uint8_t* ptr = &tx_buf_[TX_BATCH_HEADER_BYTES];
        for (size_t i = 0; i < count; ++i)
        {
            Sample s;
            if (tx_queue_.pop(s))
            {
                memcpy(ptr, &s, sizeof(Sample));
                ptr += sizeof(Sample);
            }
        }

        uint16_t total_bytes =
            static_cast<uint16_t>(TX_BATCH_HEADER_BYTES + count * sizeof(Sample));
        ++tx_seq_;

        CDC_Transmit_FS(tx_buf_, total_bytes);
    }
};

inline UsbComm g_usb_comm;

} // namespace platform
