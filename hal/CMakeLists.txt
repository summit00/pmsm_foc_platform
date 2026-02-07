add_library(hal_impl STATIC
  src/tick_hal.cpp
  src/gpio_out_hal.cpp
)

set(CUBE_DIR ${CMAKE_SOURCE_DIR}/cubeMx)

target_include_directories(hal_impl PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/app/include

  ${CUBE_DIR}/Core/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
  ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
  ${CUBE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(hal_impl PUBLIC
  STM32G431xx
  USE_HAL_DRIVER
)

target_link_libraries(hal_impl PUBLIC
  stm32_mcu
)
