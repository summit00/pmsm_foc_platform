add_library(stm32_mcu INTERFACE)

target_compile_options(stm32_mcu INTERFACE
  -mcpu=cortex-m7 
  -mthumb
  -mfpu=fpv5-d16 
  -mfloat-abi=hard
  -ffunction-sections
  -fdata-sections
)

target_link_options(stm32_mcu INTERFACE
  -mcpu=cortex-m7 
  -mthumb
  -mfpu=fpv5-d16 
  -mfloat-abi=hard
  -Wl,--gc-sections
)

target_link_options(stm32_mcu INTERFACE
    --specs=nano.specs
    --specs=nosys.specs
)
