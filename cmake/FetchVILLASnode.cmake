include(FetchContent)
FetchContent_Declare(villas-node-module
	GIT_REPOSITORY https://github.com/VILLASframework/node.git
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(villas-node-module)
message(STATUS "Path to villas-node-module: " ${villas-node-module_SOURCE_DIR})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# When adding VILLASnode via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the VILLASnode target 'villas'
set(VILLASNODE_LIBRARY villas)
set(VILLASNODE_LIBRARIES ${VILLASNODE_LIBRARY})
set(VILLASNODE_INCLUDE_DIRS "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VILLASnode DEFAULT_MSG VILLASNODE_LIBRARY)
