include(FetchContent)
FetchContent_Declare(
  villas-dpsim
  GIT_REPOSITORY https://github.com/dpsim-simulator/villas-dpsim.git
  GIT_TAG        main
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(villas-dpsim)
  if(NOT villas-dpsim_POPULATED)
    FetchContent_Populate(villas-dpsim)
    add_subdirectory(${villas-dpsim_SOURCE_DIR}/examples)
  endif()
else()
  FetchContent_MakeAvailable(villas-dpsim)
  add_subdirectory(${villas-dpsim_SOURCE_DIR}/examples)
endif()

FetchContent_GetProperties(villas-dpsim)
message(STATUS "Path to villas-dpsim: " ${villas-dpsim_SOURCE_DIR})