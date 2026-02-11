#pragma once
#include <cstdint>
#include "interfaces.hpp"

namespace app {

class Heartbeat {
public:
  explicit Heartbeat(uint32_t period_ms) : period_ms_(period_ms) {}

  void start(ITickMs& tick) { last_ms_ = tick.now_ms(); }

  void update(ITickMs& tick, IDigitalOut& out)
  {
    const uint32_t now = tick.now_ms();
    if ((now - last_ms_) >= period_ms_) {
      out.toggle();
      last_ms_ = now;
    }
  }

private:
  uint32_t period_ms_;
  uint32_t last_ms_{0};
};

} // namespace app
