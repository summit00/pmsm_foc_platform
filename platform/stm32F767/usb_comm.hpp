#pragma once
#include "user_interface.hpp"
#include <cstdint>
#include <cstring>

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

static constexpr uint16_t USB_RX_MAGIC = 0xABCDu;
static constexpr uint16_t USB_TX_MAGIC = 0xDCBAu;
static constexpr uint16_t USB_PAYLOAD_N = 10u;
static constexpr uint16_t USB_FRAME_BYTES = 4u + USB_PAYLOAD_N * 4u; // 44

// External symbols from usbd_cdc_if.c — not modified, just referenced.
extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" USBD_HandleTypeDef hUsbDeviceFS;

// ---------------------------------------------------------------------------
class UsbComm
{
  public:
    // Call once after MX_USB_DEVICE_Init().
    void init()
    {
        tx_seq_ = 0;
        last_rx_seq_ = 0xFFFFu; // force first frame to be accepted
    }

    // Call from the main loop at ~1 ms.
    // 1. Poll for a newly received RX frame → apply commands to ui.
    // 2. Snapshot telemetry from ui → transmit TX frame.
    void update(app::UserInterface& ui)
    {
        poll_rx(ui);
        send_telemetry(ui);
    }

  private:
    uint16_t tx_seq_ = 0;
    uint16_t last_rx_seq_ = 0;
    uint8_t tx_buf_[USB_FRAME_BYTES] = {};

    // -----------------------------------------------------------------------
    // RX — the USB CDC stack writes received bytes directly into UserRxBufferFS
    // and calls CDC_Receive_FS (static in usbd_cdc_if.c, not patchable).
    // Instead, we reach into the CDC class handle to check whether a new
    // OUT transfer completed since the last poll.
    //
    // USBD_CDC_HandleTypeDef::RxLength is set by the stack just before calling
    // CDC_Receive_FS; CDC_Receive_FS re-arms the RX endpoint and returns.
    // We detect a new frame by comparing the cached sequence field in the
    // frame header — if it changed, we have fresh data.
    //
    // This is main-loop only (no IRQ), so UserRxBufferFS access is safe as
    // long as we treat it as volatile (USB IRQ may be writing concurrently).
    void poll_rx(app::UserInterface& ui)
    {
        if (hUsbDeviceFS.pClassData == nullptr)
            return;

        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);

        // The USB OUT transfer has completed when RxLength > 0 and the endpoint
        // is re-armed (RxState == 0 after CDC_Receive_FS re-called SetRxBuffer).
        // We use the frame sequence number as the change detector instead of a
        // state flag, because CDC_Receive_FS itself resets the endpoint.
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
        if (magic != USB_RX_MAGIC)
            return;

        // Deduplicate by sequence number: skip frames we already processed.
        uint16_t seq;
        memcpy(&seq, &local[2], 2);
        if (seq == last_rx_seq_)
            return;
        last_rx_seq_ = seq;

        // Parse payload (little-endian int32_t array).
        int32_t p[USB_PAYLOAD_N];
        memcpy(p, &local[4], USB_PAYLOAD_N * sizeof(int32_t));

        // Apply commands — written from main loop, read by ADC IRQ.
        // __disable_irq guard keeps the struct update atomic enough:
        // the ADC ISR reads these at most every 50 µs; a struct copy here
        // takes ~10 cycles, well within one instruction window.
        __disable_irq();
        ui.mEnable = static_cast<uint8_t>(p[0] != 0 ? 1u : 0u);
        ui.mMode = static_cast<uint8_t>(p[1] & 0xFFu);
        ui.targetSpeed_rpm = static_cast<float>(p[2]) * 0.01f;
        ui.mAcceleration_rpm_s = static_cast<float>(p[3]) * 0.01f;
        ui.mIsAbs_mA = static_cast<float>(p[4]) * 0.1f;
        __enable_irq();
    }

    // -----------------------------------------------------------------------
    // TX — snapshot telemetry and send a binary frame.
    // CDC_Transmit_FS is non-blocking; returns USBD_BUSY if the previous IN
    // transfer is still in flight — we silently skip that tick (~1 ms later).
    void send_telemetry(const app::UserInterface& ui)
    {
        // Snapshot atomically: ADC IRQ writes the telemetry fields at 20 kHz.
        // A brief __disable_irq prevents reading a half-written float.
        app::UserInterface snap;
        __disable_irq();
        snap = ui;
        __enable_irq();

        int32_t p[USB_PAYLOAD_N];
        p[0] = static_cast<int32_t>(snap.actualSpeed_rpm);
        p[1] = static_cast<int32_t>(snap.Udc_V * 1000.0f);
        p[2] = static_cast<int32_t>(snap.Id_A * 1000.0f);
        p[3] = static_cast<int32_t>(snap.Iq_A * 1000.0f);
        p[4] = static_cast<int32_t>(snap.IdRef_A * 1000.0f);
        p[5] = static_cast<int32_t>(snap.IqRef_A * 1000.0f);
        p[6] = static_cast<int32_t>(snap.ThetaEncoder_deg * 100.0f);
        p[7] = static_cast<int32_t>(snap.ThetaOpenLoop_deg * 100.0f);
        p[8] = static_cast<int32_t>(5.0f * 100.0f);
        p[9] = static_cast<int32_t>(1.0f * 1000.0f);

        uint16_t magic = USB_TX_MAGIC;
        memcpy(&tx_buf_[0], &magic, 2);
        memcpy(&tx_buf_[2], &tx_seq_, 2);
        memcpy(&tx_buf_[4], p, USB_PAYLOAD_N * sizeof(int32_t));
        ++tx_seq_;

        CDC_Transmit_FS(tx_buf_, USB_FRAME_BYTES);
    }
};

inline UsbComm g_usb_comm;

} // namespace platform
