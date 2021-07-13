# Workaround for spdlog on Windows and as module

option(SPDLOG_BUILD_TESTING "Build spdlog tests" OFF)
option(SPDLOG_BUILD_BENCH "Build spdlog benchmarks" OFF)
option(SPDLOG_BUILD_EXAMPLES "Build spdlog examples" OFF)

#set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/spdlog/include)

include(FetchContent)
FetchContent_Declare(
  spdlog-module
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.5.0
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(spdlog-module)
  if(NOT spdlog-module_POPULATED)
    FetchContent_Populate(spdlog-module)
    add_subdirectory(${spdlog-module_SOURCE_DIR} ${spdlog-module_BINARY_DIR})
  endif()
else()
  FetchContent_MakeAvailable(spdlog-module)
endif()

FetchContent_GetProperties(spdlog-module)
message(STATUS "Path to spdlog-module: " ${spdlog-module_SOURCE_DIR})

set(SPDLOG_INCLUDE_DIR ${spdlog-module_SOURCE_DIR}/include)