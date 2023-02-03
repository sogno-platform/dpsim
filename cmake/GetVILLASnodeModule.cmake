
include(FetchContent)
FetchContent_Declare(
  villasnode-module
  GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/villas/node.git
  GIT_TAG        master
  GIT_PROGRESS TRUE
)

FetchContent_GetProperties(villasnode-module)
if(NOT villasnode-module_POPULATED)
  FetchContent_Populate(villasnode-module)
endif()

FetchContent_GetProperties(villasnode-module)
message(STATUS "Path to villasnode-module: " ${villasnode-module_SOURCE_DIR})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# When adding VILLASnode via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the VILLASnode target 'villas'
add_subdirectory(${villasnode-module_SOURCE_DIR})
set(VILLASnode_LIBRARY villas)
set(VILLASnode_LIBRARIES ${VILLASnode_LIBRARY})
set(VILLASnode_INCLUDE_DIRS "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VILLASnode DEFAULT_MSG VILLASnode_LIBRARY)
