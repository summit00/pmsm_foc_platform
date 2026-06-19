add_library(bsp_board INTERFACE)

set(CUBE_DIR ${CMAKE_SOURCE_DIR}/cubeMx/${TARGET_BOARD})

if(TARGET_BOARD STREQUAL "nucleoG431_ihm07")
  target_include_directories(bsp_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32g431
    ${CMAKE_SOURCE_DIR}/app
    ${CUBE_DIR}/Core/Inc
    ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CUBE_DIR}/Drivers/CMSIS/Include
  )

  target_compile_definitions(bsp_board INTERFACE
    STM32G431xx
    USE_HAL_DRIVER
  )
elseif(TARGET_BOARD STREQUAL "nucleoF767_ihm07")
  target_include_directories(bsp_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f767
    ${CMAKE_SOURCE_DIR}/app
    ${CUBE_DIR}/Core/Inc
    ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${CUBE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${CUBE_DIR}/Drivers/CMSIS/Include
  )

  target_compile_definitions(bsp_board INTERFACE
    STM32F767xx
    USE_HAL_DRIVER
  )
endif()

target_link_libraries(bsp_board INTERFACE
  stm32_mcu
)