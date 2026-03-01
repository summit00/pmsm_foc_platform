#pragma once
#include "interfaces.hpp"
#include <cstdint>

namespace app
{

class RuntimeMeasurement final
{
  public:
    explicit RuntimeMeasurement(const ICycleCounter& counter) : m_counter(counter)
    {
    }

    inline void start()
    {
        m_start = m_counter.now_cycles();
    }

    inline void stop()
    {
        m_end = m_counter.now_cycles();
    }

    // Raw cycle count of last measured interval
    uint32_t elapsed_cycles() const
    {
        // Handles 32-bit rollover correctly
        return m_end - m_start;
    }

    // Elapsed time in microseconds
    float elapsed_us() const
    {
        return static_cast<float>(elapsed_cycles()) /
               (static_cast<float>(m_counter.cycles_per_second()) / 1e6f);
    }

    // What fraction of a given period was consumed — useful for ISR load %
    // Pass in your ISR period in cycles: e.g. SystemCoreClock / 20000 for 20kHz
    float load_fraction(uint32_t period_cycles) const
    {
        if (period_cycles == 0)
            return 0.0f;
        return static_cast<float>(elapsed_cycles()) / static_cast<float>(period_cycles);
    }

    // Convenience: load as percentage
    float load_percent(uint32_t period_cycles) const
    {
        return load_fraction(period_cycles) * 100.0f;
    }

  private:
    const ICycleCounter& m_counter;
    uint32_t m_start = 0;
    uint32_t m_end = 0;
};

} // namespace app