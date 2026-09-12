#pragma once
#include "adc.hpp"
#include "bsp.hpp"
#include "control.hpp"
#include "dwt_cycle_counter.hpp"
#include "encoder.hpp"
#include "gate_driver_enable.hpp"
#include "inverter.hpp"
#include "motor_params.hpp"
#include "hal/stm32h523/drv8353.hpp"

extern "C"
{
#include "main.h"
#include "spi.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi3;
}

namespace platform
{

inline hal::Drv8353 gate_driver(hspi3);

inline bool init_gate_driver()
{
    hal::Drv8353::Config cfg;
    cfg.pwm_mode    = hal::Drv8353::PwmMode::PWM_3X;
    cfg.dead_time   = hal::Drv8353::DeadTime::DT_400NS;
    cfg.csa_gain    = hal::Drv8353::CsaGain::GAIN_20_VV;
    cfg.idrivep_hs  = hal::Drv8353::IdriveP::IDRIVEP_550MA;
    cfg.idriven_hs  = hal::Drv8353::IdriveN::IDRIVEN_1100MA;
    cfg.idrivep_ls  = hal::Drv8353::IdriveP::IDRIVEP_550MA;
    cfg.idriven_ls  = hal::Drv8353::IdriveN::IDRIVEN_1100MA;
    cfg.tdrive      = hal::Drv8353::Tdrive::TDRIVE_1000NS;
    cfg.vds_level   = hal::Drv8353::VdsLevel::VDS_0_600V;
    cfg.ocp_mode    = hal::Drv8353::OcpMode::LATCHED_FAULT;

    return gate_driver.init(cfg);
}

inline app::MotorParams motor_params{.Rs_ohm = 0.1f,
                                     .RTotal_ohm = 0.1f,
                                     .Ld_H = 0.00016f,
                                     .Lq_H = 0.00016f,
                                     .flux_pm_Wb = 0.00408f,
                                     .polePairs = 4.0f,
                                     .encoderOffset_ticks = 295};

inline hal::ADCSense adc_sense;
inline hal::Inverter inverter(htim1);
inline hal::EncoderQEI encoder(htim4, 2000, 4);
inline app::UserInterface ui;

inline hal::GateDriverEnable gate_enable({bsp::drv_enable().port, bsp::drv_enable().pin});

constexpr float pwmPeriod_s = 1.0f / 20000.0f;

inline app::Control control{
    adc_sense, inverter, gate_enable, encoder, motor_params, ui, pwmPeriod_s};

inline void motor_control_isr()
{
    control.run_isr();
    comm::g_telemetry_manager.capture_telemetry_isr();
}

inline void calibrate_current_sense()
{
    adc_sense.calibrate_offset();
}

inline void init_encoder()
{
    encoder.start();
}

} // namespace platform
