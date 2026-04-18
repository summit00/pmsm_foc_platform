# Unit Tests
add_executable(unit_tests
    transform_test.cpp
    dq_limiter_test.cpp
    pi_controller_test.cpp
    svm_test.cpp
)
target_link_libraries(unit_tests PRIVATE Catch2::Catch2WithMain)
target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/app)
add_test(NAME unit_tests COMMAND unit_tests)

# Simulation Tests
add_executable(sil_tests system_test.cpp)
target_link_libraries(sil_tests PRIVATE Catch2::Catch2WithMain)
target_include_directories(sil_tests PRIVATE ${CMAKE_SOURCE_DIR}/app)
add_test(NAME sil_tests COMMAND sil_tests)