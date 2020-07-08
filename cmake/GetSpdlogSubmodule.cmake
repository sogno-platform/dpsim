# Workaround for spdlog on Windows and as submodule

option(SPDLOG_BUILD_TESTING "Build spdlog tests" OFF)
option(SPDLOG_BUILD_BENCH "Build spdlog benchmarks" OFF)
option(SPDLOG_BUILD_EXAMPLES "Build spdlog examples" OFF)

#set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/spdlog/include)

include(FetchContent)
FetchContent_Declare(
  spdlog-submodule
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.5.0
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(spdlog-submodule)
  if(NOT spdlog-submodule_POPULATED)
    FetchContent_Populate(spdlog-submodule)
    add_subdirectory(${spdlog-submodule_SOURCE_DIR} ${spdlog-submodule_BINARY_DIR})
  endif()
else()
  FetchContent_MakeAvailable(spdlog-submodule)
endif()

FetchContent_GetProperties(spdlog-submodule)
message(STATUS "Path to spdlog-submodule: " ${spdlog-submodule_SOURCE_DIR})

set(SPDLOG_INCLUDE_DIR ${spdlog-submodule_SOURCE_DIR}/include)