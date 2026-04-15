// sensor.hpp
#pragma once

namespace app
{

class ISensor
{
  public:
    virtual ~ISensor() = default;

    virtual void update() = 0;

    virtual float getTheta_rad() const = 0;

    virtual float getOmega_rad_Hz() const = 0;

    virtual void reset() = 0;
};

} // namespace app