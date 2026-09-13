#pragma once
#include "interfaces.hpp"
#include <cmath>
#include <cstdint>
#include <numbers>
#include "stm32h5xx_hal.h"

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
        // 1. Set timer period to exact counts per revolution (e.g. 1999 for 2000 CPR)
        if (counts_per_rev_ > 0)
        {
            htim_->Instance->ARR = counts_per_rev_ - 1;
        }

        // 2. Configure PD14 (TIM4_CH3 Index) for Active-High 3.3V CMOS input (AM26LV32 receiver)
        GPIO_InitTypeDef GPIO_InitStruct{};
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        // 3. Configure Input Capture 3 polarity for Rising Edge (Active-High index pulse)
        TIM_IC_InitTypeDef sConfigIC{};
        sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(htim_, &sConfigIC, TIM_CHANNEL_3);

        // 4. Start Hardware Encoder and Input Capture
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

        uint16_t cnt16 = static_cast<uint16_t>(htim_->Instance->CNT);
        if (index_found_)
        {
            int32_t diff = static_cast<int32_t>(cnt16) - static_cast<int32_t>(captured_index_);
            int32_t cpr = static_cast<int32_t>(counts_per_rev_);
            int32_t wrapped = ((diff % cpr) + cpr) % cpr;
            return static_cast<uint16_t>(wrapped);
        }
        else
        {
            return cnt16 % counts_per_rev_;
        }
    }

    bool has_index() const override
    {
        return index_found_;
    }

    uint16_t get_captured_index() const override
    {
        return captured_index_;
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
    mutable uint16_t captured_index_;
    mutable bool index_found_;
};

} // namespace hal
