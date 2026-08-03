#pragma once
#include <cstdint>
#include <cstddef>
#include "comm/telemetry_registry.hpp"

#ifndef __disable_irq
// Fallback for simulation or non-STM32 platform compilation
#define __disable_irq()
#define __enable_irq()
#endif

namespace platform
{

struct __attribute__((packed)) Sample {
    uint8_t id;
    int16_t value;
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

} // namespace platform

namespace comm
{

class TelemetryManager
{
  public:
    static constexpr size_t MAX_SELECTED_IDS = 10;

    struct ActiveTelemetry
    {
        const float* value_ptr = nullptr;
        float scale = 0.0f;
        uint8_t id = 0;
    };

    void set_selected_ids(const uint16_t* ids, size_t count)
    {
        ActiveTelemetry temp_entries[MAX_SELECTED_IDS] = {};
        uint16_t temp_ids[MAX_SELECTED_IDS] = {};
        size_t n = (count > MAX_SELECTED_IDS) ? MAX_SELECTED_IDS : count;

        for (size_t i = 0; i < n; ++i)
        {
            uint16_t target_id = ids[i];
            temp_ids[i] = target_id;
            if (target_id == 0)
            {
                continue;
            }

            for (const auto& entry : platform::telemetry_registry)
            {
                if (entry.id == target_id)
                {
                    temp_entries[i].value_ptr = entry.value_ptr;
                    temp_entries[i].scale = entry.scale;
                    temp_entries[i].id = static_cast<uint8_t>(target_id);
                    break;
                }
            }
        }
        for (size_t i = n; i < MAX_SELECTED_IDS; ++i)
        {
            temp_ids[i] = 0;
        }

        __disable_irq();
        for (size_t i = 0; i < MAX_SELECTED_IDS; ++i)
        {
            selected_ids_[i] = temp_ids[i];
            active_entries_[i] = temp_entries[i];
        }
        __enable_irq();
    }

    const uint16_t* get_selected_ids() const
    {
        return selected_ids_;
    }

    void init()
    {
        uint16_t ids[MAX_SELECTED_IDS] = {};
        size_t init_count = (platform::TELEMETRY_REGISTRY_SIZE < MAX_SELECTED_IDS)
                                ? platform::TELEMETRY_REGISTRY_SIZE
                                : MAX_SELECTED_IDS;
        for (size_t i = 0; i < init_count; ++i)
        {
            ids[i] = platform::telemetry_registry[i].id;
        }
        set_selected_ids(ids, init_count);
    }

    void push_sample(const platform::Sample& sample)
    {
        tx_queue_.push(sample);
    }

    bool pop_sample(platform::Sample& sample)
    {
        return tx_queue_.pop(sample);
    }

    size_t available_samples() const
    {
        return tx_queue_.size();
    }

    // Optimized capture loop: runs in O(1) constant time without registry lookups
    void capture_telemetry_isr()
    {
        for (size_t i = 0; i < MAX_SELECTED_IDS; ++i)
        {
            const auto& t = active_entries_[i];
            if (t.value_ptr != nullptr)
            {
                tx_queue_.push({t.id, static_cast<int16_t>(*t.value_ptr * t.scale)});
            }
        }
    }

  private:
    uint16_t selected_ids_[MAX_SELECTED_IDS] = {};
    ActiveTelemetry active_entries_[MAX_SELECTED_IDS] = {};
    platform::RingBuffer<platform::Sample, 2048> tx_queue_;
};

inline TelemetryManager g_telemetry_manager;

} // namespace comm
