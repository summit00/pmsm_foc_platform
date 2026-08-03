add_executable(${PROJECT_NAME}.elf)

set(CUBE_DIR ${CMAKE_SOURCE_DIR}/cubeMx/${TARGET_BOARD})

if(TARGET_BOARD STREQUAL "nucleoG431_ihm07")
  set(MCU_FLAGS 
    -mcpu=cortex-m4 
    -mthumb
    -mfpu=fpv4-sp-d16 
    -mfloat-abi=hard
    -O2
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

  set(STARTUP_FILE  ${CUBE_DIR}/startup_stm32g431xx.s)
  set(LINKER_SCRIPT ${CUBE_DIR}/STM32G431xx_FLASH.ld)

  set(CUBE_SOURCES
    ${CUBE_DIR}/Core/Src/gpio.c
    ${CUBE_DIR}/Core/Src/adc.c
    ${CUBE_DIR}/Core/Src/tim.c
    ${CUBE_DIR}/Core/Src/dma.c
    ${CUBE_DIR}/Core/Src/usart.c
    ${CUBE_DIR}/Core/Src/stm32g4xx_hal_msp.c
    ${CUBE_DIR}/Core/Src/system_stm32g4xx.c
  )

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
    stm32G431/main.cpp
    stm32G431/irq_handler.cpp
  )

elseif(TARGET_BOARD STREQUAL "nucleoF767_ihm07")
  set(MCU_FLAGS 
    -mcpu=cortex-m7 
    -mthumb
    -mfpu=fpv5-d16 
    -mfloat-abi=hard
    -O2
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
    ${CMAKE_SOURCE_DIR}
    ${CUBE_DIR}/Core/Inc
    ${CUBE_DIR}/USB_DEVICE/App
    ${CUBE_DIR}/USB_DEVICE/Target
    ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${CUBE_DIR}/Drivers/CMSIS/Include
    ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CUBE_DIR}/Drivers/BSP/STM32F7xx_Nucleo

    ${CMAKE_CURRENT_SOURCE_DIR}/stm32F767
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    STM32F767xx
    USE_HAL_DRIVER
  )

  if(EXISTS "${CUBE_DIR}/Core/Src/main.c")
    set(STARTUP_FILE  ${CUBE_DIR}/startup_stm32f767xx.s)
    set(LINKER_SCRIPT ${CUBE_DIR}/STM32F767xx_FLASH.ld)

    set_source_files_properties(${CUBE_DIR}/Core/Src/main.c PROPERTIES COMPILE_DEFINITIONS "main=cube_main")

    set(CUBE_SOURCES
      ${CUBE_DIR}/Core/Src/gpio.c
      ${CUBE_DIR}/Core/Src/adc.c
      ${CUBE_DIR}/Core/Src/tim.c
      ${CUBE_DIR}/Core/Src/stm32f7xx_hal_msp.c
      ${CUBE_DIR}/Core/Src/system_stm32f7xx.c
    )

    set(HAL_SOURCES
      ${CUBE_DIR}/Core/Src/main.c
      ${CUBE_DIR}/Core/Src/system_stm32f7xx.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
      ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
    )

    set(USB_SOURCES
      ${CUBE_DIR}/USB_DEVICE/Target/usbd_conf.c
      ${CUBE_DIR}/USB_DEVICE/App/usb_device.c
      ${CUBE_DIR}/USB_DEVICE/App/usbd_desc.c
      ${CUBE_DIR}/USB_DEVICE/App/usbd_cdc_if.c
      ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
      ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
      ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
      ${CUBE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    )

    target_sources(${PROJECT_NAME}.elf PRIVATE
      ${CUBE_SOURCES}
      ${HAL_SOURCES}
      ${USB_SOURCES}
      ${STARTUP_FILE}
      stm32F767/main.cpp
      stm32F767/irq_handler.cpp
    )

    target_link_options(${PROJECT_NAME}.elf PRIVATE
      -T${LINKER_SCRIPT}
      -Wl,-Map=${PROJECT_NAME}.map
    )
  else()
    message(WARNING "CubeMX files for ${TARGET_BOARD} not found in ${CUBE_DIR}. Please generate files in STM32CubeMX first.")
    
    # Still compile the user code to avoid empty target errors
    target_sources(${PROJECT_NAME}.elf PRIVATE
      stm32F767/main.cpp
      stm32F767/irq_handler.cpp
    )
  endif()
endif()

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
