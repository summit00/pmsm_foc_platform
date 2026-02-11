#pragma once
#include <cstdint>

namespace app {

struct ITickMs {
  virtual ~ITickMs() = default;
  virtual uint32_t now_ms() const = 0;
};

struct IDigitalOut {
  virtual ~IDigitalOut() = default;
  virtual void toggle() = 0;
};

} // namespace app
