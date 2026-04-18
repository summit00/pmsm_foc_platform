# Create the test executable
add_executable(system_test
    system_test.cpp
)

# Tell the compiler where to find the simulation and mock hardware headers
target_include_directories(system_test PRIVATE
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/simulation
    ${CMAKE_SOURCE_DIR}/hal/sim
    ${CMAKE_SOURCE_DIR}/platform/sim
)

# Link the Catch2 testing framework
target_link_libraries(system_test PRIVATE 
    Catch2::Catch2WithMain
)

# If your 'app' folder generates a CMake library target (e.g., add_library(app ...)), 
# uncomment the line below to link it!
# target_link_libraries(system_test PRIVATE app)

# Register the test so CTest / VS Code Test Explorer can find it
include(Catch)
catch_discover_tests(system_test)