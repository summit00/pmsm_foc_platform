#pragma once
#include "app/interfaces.hpp"

namespace hal {

class TickHal final : public app::ITickMs {
public:
  uint32_t now_ms() const override;
};

} // namespace hal
