FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v2.4.0
  )
FetchContent_MakeAvailable(nanobind)

if(CMAKE_VERSION VERSION_LESS 3.18)
set(DEV_MODULE Development)
else()
set(DEV_MODULE Development.Module)
endif()
find_package(Python 3.11 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)
include_directories(${Python_INCLUDE_DIRS})
message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")
message(STATUS "Python_VERSION: ${Python_VERSION}")
message(STATUS "PYTHON INCLUDE DIRECTORIES" ${Python_INCLUDE_DIRS})

nanobind_add_module(ismpc_py ${PROJECT_SOURCE_DIR}/bindings/ismpc_binding.cpp)
target_include_directories(ismpc_py PUBLIC ${eigen_SOURCE_DIR} ${ISMPC_INCLUDE_DIR})
target_link_libraries(ismpc_py PUBLIC ${ISMPC_CPP_LIBRARY})
