#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "hal/itransfer.hpp"

extern "C"
{
#include "ux_api.h"
#include "ux_device_descriptors.h"
#include "ux_device_cdc_acm.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"
#include "usb.h"
UINT _ux_dcd_stm32_initialize(ULONG dcd_io, ULONG parameter);
}

namespace hal
{

class UsbxCdcTransfer : public ITransfer
{
public:
    UsbxCdcTransfer() = default;

    bool init(PCD_HandleTypeDef& hpcd)
    {
        // 1. Ensure USB interrupt is disabled during configuration
        HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);

        // 2. Enable VDDUSB power supply for USB PHY
        HAL_PWREx_EnableVddUSB();

        // 3. Enable and configure Clock Recovery System (CRS) for HSI48 USB synchronization
        __HAL_RCC_CRS_CLK_ENABLE();
        RCC_CRSInitTypeDef crs_init{};
        crs_init.Prescaler = RCC_CRS_SYNC_DIV1;
        crs_init.Source = RCC_CRS_SYNC_SOURCE_USB;
        crs_init.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
        crs_init.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
        crs_init.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
        crs_init.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
        HAL_RCCEx_CRSConfig(&crs_init);

        // 4. Initialize USBX Memory Pool (32 KB)
        alignas(4) static uint8_t usbx_memory_pool[32768];
        if (ux_system_initialize(usbx_memory_pool, sizeof(usbx_memory_pool), UX_NULL, 0) != UX_SUCCESS)
        {
            return false;
        }

        // 5. USB Descriptors (USB-IF compliant CDC-ACM with IAD, PID 0x5740 ST Virtual COM Port)
        alignas(4) static const uint8_t cdc_device_framework_fs[] = {
            // Device Descriptor (18 bytes)
            0x12, 0x01, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40,
            0x83, 0x04, 0x40, 0x57, 0x00, 0x02, 0x01, 0x02, 0x03, 0x01,

            // Configuration Descriptor (9 bytes, total length 75 bytes)
            0x09, 0x02, 0x4B, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,

            // Interface Association Descriptor (8 bytes)
            0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,

            // Interface 0: CDC Communication (9 bytes)
            0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

            // Header Functional Descriptor (5 bytes)
            0x05, 0x24, 0x00, 0x10, 0x01,

            // Call Management Functional Descriptor (5 bytes)
            0x05, 0x24, 0x01, 0x00, 0x01,

            // ACM Functional Descriptor (4 bytes)
            0x04, 0x24, 0x02, 0x02,

            // Union Functional Descriptor (5 bytes)
            0x05, 0x24, 0x06, 0x00, 0x01,

            // Endpoint 1: Interrupt IN (Cmd) (7 bytes)
            0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10,

            // Interface 1: CDC Data (9 bytes)
            0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

            // Endpoint 3: Bulk OUT (Data) (7 bytes)
            0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,

            // Endpoint 2: Bulk IN (Data) (7 bytes)
            0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00
        };

        alignas(4) static const uint8_t cdc_language_id_framework[] = {
            0x09, 0x04 // 0x0409: US English
        };

        alignas(4) static const uint8_t cdc_string_framework[] = {
            // String 1: Manufacturer ("STMicroelectronics")
            0x09, 0x04, 0x01, 18,
            'S', 'T', 'M', 'i', 'c', 'r', 'o', 'e', 'l', 'e', 'c', 't', 'r', 'o', 'n', 'i', 'c', 's',

            // String 2: Product ("STM32 Virtual COM")
            0x09, 0x04, 0x02, 17,
            'S', 'T', 'M', '3', '2', ' ', 'V', 'i', 'r', 't', 'u', 'a', 'l', ' ', 'C', 'O', 'M',

            // String 3: Serial ("000000000001")
            0x09, 0x04, 0x03, 12,
            '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'
        };

        // 6. Initialize USBX Device Stack
        if (ux_device_stack_initialize(
                const_cast<uint8_t*>(cdc_device_framework_fs), sizeof(cdc_device_framework_fs),
                const_cast<uint8_t*>(cdc_device_framework_fs), sizeof(cdc_device_framework_fs),
                const_cast<uint8_t*>(cdc_string_framework), sizeof(cdc_string_framework),
                const_cast<uint8_t*>(cdc_language_id_framework), sizeof(cdc_language_id_framework),
                USBD_ChangeFunction) != UX_SUCCESS)
        {
            return false;
        }

        // 7. Register CDC ACM Class with instance tracking callbacks
        static UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm_parameter;
        cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = cdc_activate_cb;
        cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = cdc_deactivate_cb;
        cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change    = USBD_CDC_ACM_ParameterChange;

        if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                           ux_device_class_cdc_acm_entry,
                                           1, 0,
                                           &cdc_acm_parameter) != UX_SUCCESS)
        {
            return false;
        }

        // 8. Initialize PCD hardware
        MX_USB_PCD_Init();
        HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn); // MspInit enables IRQ; disable until DCD is attached

        // 9. Configure PMA (Packet Memory Area) buffer offsets for STM32 DRD FS
        // PMA total size is 2 KB. The first 64 bytes (0x00-0x3F) hold the 8 Buffer Descriptors (TXBD/RXBD).
        // Each endpoint buffer must be mapped above 0x40 with no overlaps.
        HAL_PCDEx_PMAConfig(&hpcd, 0x00, PCD_SNG_BUF, 0x40);  // EP0 OUT (Control OUT, 64 bytes: 0x40-0x7F)
        HAL_PCDEx_PMAConfig(&hpcd, 0x80, PCD_SNG_BUF, 0x80);  // EP0 IN  (Control IN,  64 bytes: 0x80-0xBF)
        HAL_PCDEx_PMAConfig(&hpcd, 0x81, PCD_SNG_BUF, 0xC0);  // EP1 IN  (CDC CMD Interrupt IN, 64 bytes: 0xC0-0xFF)
        HAL_PCDEx_PMAConfig(&hpcd, 0x82, PCD_SNG_BUF, 0x100); // EP2 IN  (CDC Data Bulk IN, 64 bytes: 0x100-0x13F)
        HAL_PCDEx_PMAConfig(&hpcd, 0x03, PCD_SNG_BUF, 0x140); // EP3 OUT (CDC Data Bulk OUT, 64 bytes: 0x140-0x17F)

        // 10. Bind STM32 DCD controller to USBX device stack
        if (_ux_dcd_stm32_initialize((ULONG)hpcd.Instance, (ULONG)&hpcd) != UX_SUCCESS)
        {
            return false;
        }

        // 11. Start PCD with soft-reconnect
        HAL_PCD_DevDisconnect(&hpcd);
        HAL_Delay(10);
        HAL_PCD_Start(&hpcd);

        // 12. Enable USB Interrupt (Priority 1: just below 20 kHz ADC IRQ at priority 0)
        HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

        return true;
    }

    void poll_tasks()
    {
        // Continuously run USBX background tasks (DCD state machine, enumeration, class tasks)
        ux_device_stack_tasks_run();
    }

    bool send(const uint8_t* data, size_t len) override
    {
        auto* cdc = get_cdc_instance();
        if (cdc == nullptr)
        {
            return false;
        }

        ULONG actual_length = 0;
        UINT status = ux_device_class_cdc_acm_write_run(
            cdc, const_cast<UCHAR*>(data), static_cast<ULONG>(len), &actual_length);

        return (status == UX_STATE_NEXT || status == UX_STATE_WAIT || status == UX_SUCCESS);
    }

    bool is_tx_busy() const override
    {
        auto* cdc = get_cdc_instance();
        if (cdc == nullptr)
        {
            return false;
        }
        if (cdc->ux_device_class_cdc_acm_write_state != 0)
        {
            ULONG actual_written = 0;
            ux_device_class_cdc_acm_write_run(cdc, nullptr, 0, &actual_written);
        }
        return (cdc->ux_device_class_cdc_acm_write_state != 0);
    }

    void set_rx_callback(RxCallback callback, void* ctx) override
    {
        rxCb_ = callback;
        rxCtx_ = ctx;
    }

    void poll_rx()
    {
        // 1. Run USBX background state machines for device stack & class
        ux_device_stack_tasks_run();

        auto* cdc = get_cdc_instance();
        if (cdc == nullptr)
        {
            return;
        }

        // If a write is in progress, advance the write state machine
        if (cdc->ux_device_class_cdc_acm_write_state != 0)
        {
            ULONG actual_written = 0;
            ux_device_class_cdc_acm_write_run(cdc, nullptr, 0, &actual_written);
        }

        if (rxCb_ == nullptr)
        {
            return;
        }

        // 2. Poll for received bytes from USB host
        ULONG actual_length = 0;
        UINT status = ux_device_class_cdc_acm_read_run(
            cdc, rx_buf_, sizeof(rx_buf_), &actual_length);

        if ((status == UX_STATE_NEXT || status == UX_SUCCESS) && actual_length > 0)
        {
            rxCb_(rx_buf_, actual_length, rxCtx_);
        }
    }

private:
    inline static UX_SLAVE_CLASS_CDC_ACM* s_cdc_instance = nullptr;

    static VOID cdc_activate_cb(VOID* instance)
    {
        s_cdc_instance = static_cast<UX_SLAVE_CLASS_CDC_ACM*>(instance);
        USBD_CDC_ACM_Activate(instance);
    }

    static VOID cdc_deactivate_cb(VOID* instance)
    {
        s_cdc_instance = nullptr;
        USBD_CDC_ACM_Deactivate(instance);
    }

    static UINT USBD_ChangeFunction(ULONG /*Device_State*/)
    {
        return UX_SUCCESS;
    }

    static UX_SLAVE_CLASS_CDC_ACM* get_cdc_instance()
    {
        if (s_cdc_instance != nullptr)
        {
            return s_cdc_instance;
        }
        if (_ux_system_slave == nullptr)
        {
            return nullptr;
        }
        if (_ux_system_slave->ux_system_slave_device.ux_slave_device_state != UX_DEVICE_CONFIGURED)
        {
            return nullptr;
        }

        auto* interface_ptr = _ux_system_slave->ux_system_slave_device.ux_slave_device_first_interface;
        while (interface_ptr != nullptr)
        {
            if (interface_ptr->ux_slave_interface_class_instance != nullptr)
            {
                return static_cast<UX_SLAVE_CLASS_CDC_ACM*>(interface_ptr->ux_slave_interface_class_instance);
            }
            interface_ptr = interface_ptr->ux_slave_interface_next_interface;
        }
        return nullptr;
    }

    RxCallback rxCb_ = nullptr;
    void* rxCtx_ = nullptr;
    uint8_t rx_buf_[64]{0};
};

} // namespace hal
