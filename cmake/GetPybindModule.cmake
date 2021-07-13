# Build pybind from github repository

include(FetchContent)
FetchContent_Declare(
  pybind-module
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.5
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(pybind-module)
  if(NOT pybind-module_POPULATED)
    FetchContent_Populate(pybind-module)
    add_subdirectory(${pybind-module_SOURCE_DIR} ${pybind-module_BINARY_DIR})
  endif()
else()
  FetchContent_MakeAvailable(pybind-module)
endif()

FetchContent_GetProperties(pybind-module)
message(STATUS "Path to pybind-module: " ${pybind-module_SOURCE_DIR})