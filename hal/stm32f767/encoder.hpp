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
        : htim_(&htim), counts_per_rev_(counts_per_rev), pole_pairs_(pole_pairs),
          captured_index_(0), index_found_(false)
    {
    }

    void start()
    {
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
        HAL_TIM_IC_Start(htim_, TIM_CHANNEL_3);
    }

    uint16_t read_raw() const override
    {
        // Check if index capture occurred on TIM4 Channel 3
        if (__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC3))
        {
            captured_index_ = static_cast<uint16_t>(htim_->Instance->CCR3);
            index_found_ = true;
            __HAL_TIM_CLEAR_FLAG(htim_, TIM_FLAG_CC3);
        }

        // TIM4 is 16-bit counter.
        uint16_t cnt16 = static_cast<uint16_t>(htim_->Instance->CNT);
        if (index_found_)
        {
            // Calculate distance since index captured (handles timer wrap-around perfectly)
            uint16_t ticks_since_index = static_cast<uint16_t>(cnt16 - captured_index_);
            return ticks_since_index % counts_per_rev_;
        }
        else
        {
            // Fallback before first index pulse is seen
            return cnt16 % counts_per_rev_;
        }
    }

    void reset() override
    {
        htim_->Instance->CNT = 0;
        index_found_ = false;
        captured_index_ = 0;
    }

  private:
    TIM_HandleTypeDef* htim_;
    uint16_t counts_per_rev_;
    uint16_t pole_pairs_;
    float rad_per_count_;
    mutable uint16_t captured_index_;
    mutable bool index_found_;
};

} // namespace hal