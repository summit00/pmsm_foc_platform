cmake_minimum_required(VERSION 3.20)

project(pmsm_foc_platform LANGUAGES C CXX ASM)

# Add this global definition
add_definitions(-DUSE_NUCLEO_64)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PLATFORM "stm32" CACHE STRING "Build platform: host|stm32")
set_property(CACHE PLATFORM PROPERTY STRINGS host stm32)

option(BUILD_TESTS "Build unit tests (host only)" ON)

add_subdirectory(app)

if(PLATFORM STREQUAL "host")
    set(BUILD_TESTS ON CACHE BOOL "" FORCE)
    enable_testing()
    add_subdirectory(third_party/Catch2)
    add_subdirectory(tests)

elseif(PLATFORM STREQUAL "stm32")
    include(${CMAKE_SOURCE_DIR}/cmake/stm32g431.cmake)
    add_subdirectory(hal)
    add_subdirectory(bsp)
    add_subdirectory(platform)

else()
    message(FATAL_ERROR "Unknown platform: ${PLATFORM}")
endif()
