include(FetchContent)
FetchContent_Declare(
  cim-data
  GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/grid-data/cim-grid-data.git
  GIT_TAG        master
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(cim-data)
  if(NOT cim-data_POPULATED)
    FetchContent_Populate(cim-data)
  endif()
else()
  FetchContent_MakeAvailable(cim-data)
endif()

FetchContent_GetProperties(cim-data)
message(STATUS "Path to cim-data: " ${cim-data_SOURCE_DIR})

FetchContent_Declare(
  profile-data
  GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/grid-data/grid-profiles.git
  GIT_TAG        master
)

if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(profile-data)
  if(NOT profile-data_POPULATED)
    FetchContent_Populate(profile-data)
  endif()
else()
  FetchContent_MakeAvailable(profile-data)
endif()

FetchContent_GetProperties(profile-data)
message(STATUS "Path to profile-data: " ${profile-data_SOURCE_DIR})