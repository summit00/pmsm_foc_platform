cmake_minimum_required(VERSION 3.20)

project(pmsm_foc_platform LANGUAGES C CXX ASM)

# Add this global definition
add_definitions(-DUSE_NUCLEO_64)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_SOURCE_DIR}/cmake/stm32g431.cmake)

add_subdirectory(app)
add_subdirectory(hal)
add_subdirectory(bsp)
add_subdirectory(platform)