#pragma once
#include "encoder.hpp"
#include <cmath>
#include <cstdint>
#include <numbers>

extern "C"
{
#include "tim.h"
}

namespace hal
{

class EncoderQEI : public app::IEncoder
{
  public:
    explicit EncoderQEI(TIM_HandleTypeDef& htim, uint16_t counts_per_rev, uint16_t pole_pairs)
        : htim_(&htim), counts_per_rev_(counts_per_rev), pole_pairs_(pole_pairs)
    {
    }

    void start()
    {
        HAL_TIM_Encoder_Start(&(*htim_), TIM_CHANNEL_ALL);
    }

    uint16_t read_raw() const override
    {
        // TIM2 is 32-bit counter.
        uint32_t cnt32 = htim_->Instance->CNT;
        // Modulo to counts_per_rev to get position within one revolution.
        return static_cast<uint16_t>(cnt32 % counts_per_rev_);
    }

    void reset() override
    {
        htim_->Instance->CNT = 0;
    }

  private:
    TIM_HandleTypeDef* htim_;
    uint16_t counts_per_rev_;
    uint16_t pole_pairs_;
    float rad_per_count_;
};

} // namespace hal