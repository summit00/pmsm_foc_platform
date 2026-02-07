add_library(stm32_mcu INTERFACE)

target_compile_options(stm32_mcu INTERFACE
  -mcpu=cortex-m4 
  -mthumb
  -mfpu=fpv4-sp-d16 
  -mfloat-abi=hard
  -ffunction-sections
  -fdata-sections
)

target_link_options(stm32_mcu INTERFACE
  -mcpu=cortex-m4 
  -mthumb
  -mfpu=fpv4-sp-d16 
  -mfloat-abi=hard
  -Wl,--gc-sections
)

target_link_options(stm32_mcu INTERFACE
    --specs=nano.specs
    --specs=nosys.specs
)