#pragma once
#include <cstring>
#include "hal/itransfer.hpp"

extern "C"
{
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
}

extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" USBD_HandleTypeDef hUsbDeviceFS;

namespace hal
{

class UsbCdcTransfer : public ITransfer
{
  public:
    UsbCdcTransfer() = default;

    bool send(const uint8_t* data, size_t len) override
    {
        if (hUsbDeviceFS.pClassData == nullptr)
        {
            return false;
        }

        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
        if (hcdc->TxState != 0)
        {
            return false; // Busy
        }

        // CDC_Transmit_FS casts constness away internally
        uint8_t status = CDC_Transmit_FS(const_cast<uint8_t*>(data), static_cast<uint16_t>(len));
        return status == USBD_OK;
    }

    bool is_tx_busy() const override
    {
        if (hUsbDeviceFS.pClassData == nullptr)
        {
            return false;
        }
        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
        return hcdc->TxState != 0;
    }

    void set_rx_callback(RxCallback callback, void* ctx) override
    {
        rxCb_ = callback;
        rxCtx_ = ctx;
    }

    // Call from background thread to poll CDC rx buffer
    void poll_rx()
    {
        if (hUsbDeviceFS.pClassData == nullptr || rxCb_ == nullptr)
        {
            return;
        }

        auto* hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
        uint16_t rx_len = static_cast<uint16_t>(hcdc->RxLength);
        if (rx_len < 44) // Check if at least 44-byte frame has been received
        {
            return;
        }

        uint8_t local[44];
        __disable_irq();
        std::memcpy(local, UserRxBufferFS, 44);
        __enable_irq();

        rxCb_(local, 44, rxCtx_);
    }

  private:
    RxCallback rxCb_ = nullptr;
    void* rxCtx_ = nullptr;
};

} // namespace hal
