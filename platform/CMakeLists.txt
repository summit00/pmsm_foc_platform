add_executable(${PROJECT_NAME}.elf)

set(CUBE_DIR ${CMAKE_SOURCE_DIR}/cubeMx)


# STM32G431 = Cortex-M4F
set(MCU_FLAGS 
  -mcpu=cortex-m4 
  -mthumb
  -mfpu=fpv4-sp-d16 
  -mfloat-abi=hard
  -O3
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -Wall -Wextra
  -ffunction-sections -fdata-sections
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -Wl,--gc-sections
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
  ${CUBE_DIR}/Core/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
  ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
  ${CUBE_DIR}/Drivers/CMSIS/Include
  ${CUBE_DIR}/Drivers/BSP/STM32G4xx_Nucleo

  ${CMAKE_CURRENT_SOURCE_DIR}/stm32G431
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
  STM32G431xx
  USE_HAL_DRIVER
)

# Adjust these two filenames if your CubeMX output differs
set(STARTUP_FILE  ${CUBE_DIR}/startup_stm32g431xx.s)
set(LINKER_SCRIPT ${CUBE_DIR}/STM32G431xx_FLASH.ld)

# CubeMX sources (NO Cube main.c; platform owns main())
set(CUBE_SOURCES
  ${CUBE_DIR}/Core/Src/gpio.c
  ${CUBE_DIR}/Core/Src/adc.c
  ${CUBE_DIR}/Core/Src/tim.c
  ${CUBE_DIR}/Core/Src/dma.c
  ${CUBE_DIR}/Core/Src/usart.c
  #${CUBE_DIR}/Core/Src/stm32g4xx_it.c
  ${CUBE_DIR}/Core/Src/stm32g4xx_hal_msp.c
  ${CUBE_DIR}/Core/Src/system_stm32g4xx.c
)

# Minimal HAL driver sources for GPIO + clocks + SysTick
set(HAL_SOURCES
  ${CUBE_DIR}/Core/Src/main.c
  ${CUBE_DIR}/Core/Src/system_stm32g4xx.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
)

target_sources(${PROJECT_NAME}.elf PRIVATE
  ${CUBE_SOURCES}
  ${HAL_SOURCES}
  ${STARTUP_FILE}
  stm32g431/main.cpp
  stm32g431/irq_handler.cpp
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
  -T${LINKER_SCRIPT}
  -Wl,-Map=${PROJECT_NAME}.map
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
  app_logic
  hal_impl
  bsp_board
  stm32_mcu
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND arm-none-eabi-objcopy -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
  COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
  COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}.elf>
)
