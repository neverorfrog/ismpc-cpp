cmake_minimum_required(VERSION 3.22)

# ============================= PROJECT SETUP ===============================================
project(ismpc_cpp VERSION 0.1 LANGUAGES CXX)
set(ISMPC_CPP_LIBRARY ${PROJECT_NAME})
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
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
option(BUILD_ROS "Build ros package" OFF)

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
set(EIGEN_INCLUDE_DIR /usr/include/eigen3)

# YAML-CPP
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

# PROXSUITE
FetchContent_Declare(
  proxsuite
  GIT_REPOSITORY https://github.com/Simple-Robotics/proxsuite.git
  GIT_TAG main
)
FetchContent_MakeAvailable(proxsuite)

# WEBOTS
include(ExternalProject)
ExternalProject_Add(
  webots
  SOURCE_DIR /home/neverorfrog/code/webots # Replace with the actual path to your cloned Webots repository
  DOWNLOAD_COMMAND ""
  CONFIGURE_COMMAND "" # Skip the configure step since Webots doesn't use CMake
  BUILD_COMMAND make -j -C <SOURCE_DIR> # Adjust the build command as needed
  BUILD_IN_SOURCE true
  INSTALL_COMMAND "" # Define if there's an install step; leave empty if not applicable
)

# ============================ ISMPC LIBRARY ==================================================
set(ISMPC_CPP_EXTRA_LIBRARIES
  $<BUILD_INTERFACE:cxx_setup>
  $<BUILD_INTERFACE:yaml-cpp::yaml-cpp>
  $<BUILD_INTERFACE:proxsuite>
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
file(GLOB_RECURSE ISMPC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${ISMPC_CPP_LIBRARY} STATIC ${ISMPC_SRCS})
target_include_directories(${ISMPC_CPP_LIBRARY}
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${proxsuite_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${yaml-cpp_SOURCE_DIR}/include>
    ${EIGEN_INCLUDE_DIR}
)
target_link_libraries(${ISMPC_CPP_LIBRARY} PUBLIC ${ISMPC_CPP_EXTRA_LIBRARIES})

if(BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  if ( ament_cmake_FOUND )
    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "ismpc_cpp is being built using AMENT.")
    message(STATUS "------------------------------------------")
    set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CONFIG_PATH})

    find_package(ament_index_cpp REQUIRED)
    list( APPEND ISMPC_CPP_EXTRA_LIBRARIES $<BUILD_INTERFACE:ament_index_cpp::ament_index_cpp>)
    ament_export_dependencies(ament_index_cpp)
    ament_export_dependencies(yaml-cpp)

    set( ISMPC_CPP_LIB_DESTINATION lib )
    set( ISMPC_CPP_INCLUDE_DESTINATION include )
    set( ISMPC_CPP_BIN_DESTINATION bin )

    mark_as_advanced(
      ISMPC_CPP_EXTRA_LIBRARIES
      ISMPC_CPP_LIB_DESTINATION
      ISMPC_CPP_INCLUDE_DESTINATION
      ISMPC_CPP_BIN_DESTINATION
    )

    macro(export_ismpc_cpp_package)
      ament_export_include_directories(include)
      ament_export_libraries(${ISMPC_CPP_LIBRARY})
      ament_export_targets(${ISMPC_CPP_LIBRARY}Targets)
      ament_package()
    endmacro()
  endif()

  INSTALL(
    TARGETS ${ISMPC_CPP_LIBRARY}
    EXPORT ${ISMPC_CPP_LIBRARY}Targets
    ARCHIVE DESTINATION ${ISMPC_CPP_LIB_DESTINATION}
    LIBRARY DESTINATION ${ISMPC_CPP_LIB_DESTINATION}
    RUNTIME DESTINATION ${ISMPC_CPP_BIN_DESTINATION}
    INCLUDES DESTINATION ${ISMPC_CPP_INCLUDE_DESTINATION}
  )

  INSTALL(
      DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
        $<BUILD_INTERFACE:${proxsuite_SOURCE_DIR}/include/proxsuite>
        ${EIGEN_INCLUDE_DIR}
        $<BUILD_INTERFACE:${nanobind_SOURCE_DIR}/include/nanobind>
        $<BUILD_INTERFACE:${yaml-cpp_SOURCE_DIR}/include/yaml-cpp>
      DESTINATION ${ISMPC_CPP_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h*"
  )

  export_ismpc_cpp_package()
endif()
# =============================================================================================

# ============================ PYTHON BINDINGS =================================================
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()
# =============================================================================================

# =========================== GOOGLE TEST =====================================================
if(BUILD_TESTS)
  add_subdirectory(test)
endif()
# =============================================================================================

# ============================ EXECUTABLES ====================================================
add_executable(ismpc_main ${CMAKE_CURRENT_SOURCE_DIR}/src/cppmain.cpp)
target_link_libraries(ismpc_main PRIVATE ${ISMPC_CPP_LIBRARY})

# link_libraries("/home/neverorfrog/code/webots/lib/controller/libCppController.so")
include_directories("/home/neverorfrog/code/webots/include/controller/cpp")
add_executable(webots_main ${CMAKE_CURRENT_SOURCE_DIR}/src/webotsmain.cpp)
# =============================================================================================
