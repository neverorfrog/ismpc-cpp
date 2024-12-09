cmake_minimum_required(VERSION 3.22)

# ============================= PROJECT SETUP ===============================================
project(ismpc VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra -Wno-dev)
target_compile_features(cxx_setup INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_LOADED)
  message(STATUS "Path: ${CMAKE_CXX_COMPILER}")
  message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/executables")
# =============================================================================================

# ============================ BUILD OPTIONS ==================================================
set(BUILD_TESTS OFF)
set(BUILD_PYTHON_BINDINGS OFF)
set(BUILD_DART OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
add_compile_definitions("DEBUG")
# endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
# =============================================================================================

# ============================ TIMESTAMP ======================================================
execute_process(
  COMMAND date "+%Y-%m-%d %H:%M:%S"
  OUTPUT_VARIABLE TIMESTAMP
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Write the timestamp to a header file
set(TIMESTAMP_HEADER "${CMAKE_BINARY_DIR}/timestamp.hpp")
file(WRITE ${TIMESTAMP_HEADER} "#pragma once\n")
file(APPEND ${TIMESTAMP_HEADER} "#define BUILD_TIMESTAMP \"${TIMESTAMP}\"\n")
# =============================================================================================

# =========================== GOOGLE TEST =====================================================
if(BUILD_TESTS)
  add_subdirectory(test)
endif()
# =============================================================================================

# ============================ EXTERNAL LIBRARIES =============================================
include(FetchContent)

# YAML-CPP
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

# PROXSUITE
FetchContent_Declare(
  proxsuite
  GIT_REPOSITORY https://github.com/Simple-Robotics/proxsuite.git
  GIT_TAG main
)
FetchContent_MakeAvailable(proxsuite)

# ============================ INCLUDE DIRECTORIES ============================================
set(ISMPC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(EIGEN_INCLUDE_DIR /usr/include/eigen3)
list(APPEND INCLUDE_DIRS ${ISMPC_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
# =============================================================================================

# ============================ LIBRARIES ======================================================
file(GLOB_RECURSE ISMPC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(ismpc ${ISMPC_SRCS})
target_include_directories(ismpc SYSTEM PUBLIC ${INCLUDE_DIRS})
target_link_libraries(ismpc PUBLIC cxx_setup yaml-cpp::yaml-cpp proxsuite)
# =============================================================================================

# ============================ EXECUTABLES ====================================================
add_executable(ismpc_main ${CMAKE_CURRENT_SOURCE_DIR}/src/cppmain.cpp)
target_link_libraries(ismpc_main PRIVATE ismpc)
# =============================================================================================

# ============================= CONFIG FILE GENERATION =======================================
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tools/systemvars.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tools/systemvars.h
  @ONLY
)
# ===========================================================================================
