#pragma once
#include "interfaces.hpp"
#include <cstdint>

namespace hal
{

class CurrentSense : public app::ICurrentSense
{
  public:
    static inline void isr_update_currents(uint16_t ia, uint16_t ib)
    {
        ia_counts = ia;
        ib_counts = ib;
    }

    inline app::PhaseCurrentsRaw read_raw() const override
    {
        return {ia_counts, ib_counts};
    }

  private:
    static inline uint16_t ia_counts = 0;
    static inline uint16_t ib_counts = 0;
};
} // namespace hal