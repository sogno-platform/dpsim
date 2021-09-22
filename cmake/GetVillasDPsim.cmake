include(FetchContent)
FetchContent_Declare(
  dpsim-villas
  GIT_REPOSITORY https://github.com/sogno-platform/dpsim-villas.git
  GIT_TAG        main
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(dpsim-villas)
  if(NOT dpsim-villas_POPULATED)
    FetchContent_Populate(dpsim-villas)
    add_subdirectory(${dpsim-villas_SOURCE_DIR})
  endif()
else()
  #Auto-calls add_subdirectory if the downloaded content contains a top-level CMakeLists.txt
  FetchContent_MakeAvailable(dpsim-villas)
endif()

FetchContent_GetProperties(dpsim-villas)
message(STATUS "Path to dpsim-villas: " ${dpsim-villas_SOURCE_DIR})