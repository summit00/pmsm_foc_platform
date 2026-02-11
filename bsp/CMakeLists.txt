add_library(bsp_board STATIC
  src/board.cpp
)

set(CUBE_DIR ${CMAKE_SOURCE_DIR}/cubeMx)

target_include_directories(bsp_board PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/app
  ${CUBE_DIR}/Core/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
  ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
  ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
  ${CUBE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(bsp_board PUBLIC
  STM32G431xx
  USE_HAL_DRIVER
)

target_link_libraries(bsp_board PUBLIC
  stm32_mcu
)