add_executable(unit_tests
    transform_test.cpp
    dq_limiter_test.cpp
    pi_controller_test.cpp
    svm_test.cpp
)

target_link_libraries(unit_tests PRIVATE
    Catch2::Catch2WithMain
    app_logic
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}
)

# Enable coverage flags if building tests
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(unit_tests PRIVATE
        $<$<CONFIG:Debug>:--coverage -fprofile-arcs -ftest-coverage>
    )
    target_link_options(unit_tests PRIVATE
        $<$<CONFIG:Debug>:--coverage -fprofile-arcs -ftest-coverage>
    )
endif()

add_test(NAME unit_tests COMMAND unit_tests)