cmake_minimum_required(VERSION 3.22)

# ============================= PROJECT SETUP ===============================================
project(ismpc VERSION 0.1 LANGUAGES CXX)
set(ISMPC_CPP_LIBRARY ismpc_cpp)
set(ISMPC_PY_LIBRARY ismpc)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra)
target_compile_features(cxx_setup INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_LOADED)
  message(STATUS "Path: ${CMAKE_CXX_COMPILER}")
  message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/executables")

# =============================================================================================

# ============================ BUILD OPTIONS ==================================================
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_ROS "Build ros package" OFF)
option(BUILD_DART "Build with DART" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions("DEBUG")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# =============================================================================================

# ============================ UTILITY STUFF ==================================================
execute_process(
  COMMAND date "+%Y-%m-%d %H:%M:%S"
  OUTPUT_VARIABLE TIMESTAMP
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Write the timestamp to a header file
set(TIMESTAMP_HEADER "${CMAKE_BINARY_DIR}/timestamp.h")
file(WRITE ${TIMESTAMP_HEADER} "#pragma once\n")
file(APPEND ${TIMESTAMP_HEADER} "#define BUILD_TIMESTAMP \"${TIMESTAMP}\"\n")

# Config file with system variables
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ismpc_cpp/tools/systemvars.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ismpc_cpp/tools/systemvars.h
  @ONLY
)

# =============================================================================================

# ============================ EXTERNAL LIBRARIES =============================================
include(FetchContent)

# EIGEN3
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
)
FetchContent_MakeAvailable(eigen)

# PROXSUITE
FetchContent_Declare(
  proxsuite
  GIT_REPOSITORY https://github.com/Simple-Robotics/proxsuite.git
  GIT_TAG v0.6.6
)
FetchContent_MakeAvailable(proxsuite)

# NANOBIND
FetchContent_Declare(
  nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind.git
  GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(nanobind)

# ============================ ISMPC LIBRARY ==================================================
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
file(GLOB_RECURSE ISMPC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM ISMPC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/bindings.cpp)
add_library(${ISMPC_CPP_LIBRARY} STATIC ${ISMPC_SRCS})

target_include_directories(${ISMPC_CPP_LIBRARY}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${proxsuite_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${eigen_SOURCE_DIR}>
  ${DART_INCLUDE_DIRS}
)

set(ISMPC_CPP_EXTRA_LIBRARIES
  $<BUILD_INTERFACE:cxx_setup>
  $<BUILD_INTERFACE:proxsuite>
  $<BUILD_INTERFACE:Eigen3::Eigen>
  ${DART_LIBRARIES}
)
target_link_libraries(${ISMPC_CPP_LIBRARY} PUBLIC ${ISMPC_CPP_EXTRA_LIBRARIES})
# =============================================================================================

# ============================ PYTHON BINDINGS =================================================
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT
)

execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('purelib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(${Python_INCLUDE_DIRS})
message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")
message(STATUS "Python_VERSION: ${Python_VERSION}")
message(STATUS "PYTHON INCLUDE DIRECTORIES" ${Python_INCLUDE_DIRS})

nanobind_add_module(${ISMPC_PY_LIBRARY} ${PROJECT_SOURCE_DIR}/src/bindings.cpp)
target_include_directories(${ISMPC_PY_LIBRARY} PUBLIC ${eigen_SOURCE_DIR} ${ISMPC_INCLUDE_DIR})
target_link_libraries(${ISMPC_PY_LIBRARY} PUBLIC ${ISMPC_CPP_LIBRARY})

install(
    TARGETS ${ISMPC_PY_LIBRARY}
    EXPORT ${ISMPC_PY_LIBRARY}Targets
    LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${BINDIR}
)

# =============================================================================================

# =========================== GOOGLE TEST =====================================================
if(BUILD_TESTS)
  add_subdirectory(test)
endif()
# =============================================================================================

# ============================ EXECUTABLES ====================================================
add_executable(ismpc_main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(ismpc_main PRIVATE ${ISMPC_CPP_LIBRARY})
# =============================================================================================