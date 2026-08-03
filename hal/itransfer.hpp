#pragma once
#include <cstdint>
#include <cstddef>

namespace hal
{

class ITransfer
{
public:
    virtual ~ITransfer() = default;

    /**
     * @brief Transmit raw bytes over the physical bus.
     * @return true if successfully queued, false if transport is busy or failed.
     */
    virtual bool send(const uint8_t* data, size_t len) = 0;

    /**
     * @brief Check if the transmission hardware is busy.
     */
    virtual bool is_tx_busy() const = 0;

    /**
     * @brief Registered callback to handle incoming bytes.
     */
    using RxCallback = void(*)(const uint8_t* data, size_t len, void* ctx);
    virtual void set_rx_callback(RxCallback callback, void* ctx) = 0;
};

} // namespace hal
