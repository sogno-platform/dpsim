# Build pybind from github repository

include(FetchContent)
FetchContent_Declare(
  pybind-submodule
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.5
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(pybind-submodule)
  if(NOT pybind-submodule_POPULATED)
    FetchContent_Populate(pybind-submodule)
    add_subdirectory(${pybind-submodule_SOURCE_DIR} ${pybind-submodule_BINARY_DIR})
  endif()
else()
  FetchContent_MakeAvailable(pybind-submodule)
endif()

FetchContent_GetProperties(pybind-submodule)
message(STATUS "Path to pybind-submodule: " ${pybind-submodule_SOURCE_DIR})