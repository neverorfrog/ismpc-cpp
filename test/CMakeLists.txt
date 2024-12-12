cmake_minimum_required(VERSION 3.22)
project(ismpc_tests)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Add test files
file(GLOB TEST_SOURCES */*.cpp)

# Create test executable
add_executable(tests ${TEST_SOURCES})

# Link GoogleTest and the main project
target_link_libraries(tests gtest gtest_main ${ISMPC_CPP_LIBRARY})

# Add tests
include(GoogleTest)
gtest_discover_tests(tests)
