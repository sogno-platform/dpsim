
include(FetchContent)
FetchContent_Declare(
  villasnode-module
  GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/villas/node.git
  GIT_TAG        b94746effb015aa98791c0e319ef11223d18e8b0
)

FetchContent_GetProperties(villasnode-module)
if(NOT villasnode-module_POPULATED)
  FetchContent_Populate(villasnode-module)
endif()

FetchContent_GetProperties(villasnode-module)
message(STATUS "Path to villasnode-module: " ${villasnode-module_SOURCE_DIR})

# When adding VILLASnode via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the VILLASnode target 'villas'
add_subdirectory(${villasnode-module_SOURCE_DIR} ${villasnode-module_BINARY_DIR})
set(VILLASnode_LIBRARY villas)
set(VILLASnode_LIBRARIES ${VILLASnode_LIBRARY})
set(VILLASnode_INCLUDE_DIR "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VILLASnode DEFAULT_MSG VILLASnode_LIBRARY)

set(VILLASnode_LIBRARIES ${VILLASnode_LIBRARY})
set(VILLASnode_INCLUDE_DIRS ${VILLASnode_INCLUDE_DIR})
