add_executable(unit_tests
    transform_test.cpp
    pi_controller_test.cpp
)

target_link_libraries(unit_tests PRIVATE app_logic Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(unit_tests)