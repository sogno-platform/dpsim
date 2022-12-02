
include(FetchContent)
FetchContent_Declare(
  villasnode-module
  # CMAKE_ARGS -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64
  GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/villas/node.git
  GIT_TAG        b94746effb015aa98791c0e319ef11223d18e8b0
  GIT_PROGRESS TRUE
)

FetchContent_GetProperties(villasnode-module)
if(NOT villasnode-module_POPULATED)
  FetchContent_Populate(villasnode-module)
  # set(CMAKE_INSTALL_LIBDIR /usr/local/lib64 CACHE INTERNAL "Specify install directory")
endif()

FetchContent_GetProperties(villasnode-module)
message(STATUS "Path to villasnode-module: " ${villasnode-module_SOURCE_DIR})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# When adding VILLASnode via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the VILLASnode target 'villas'
add_subdirectory(${villasnode-module_SOURCE_DIR} ${villasnode-module_BINARY_DIR})
set(VILLASnode_LIBRARY libvillas)
set(VILLASnode_LIBRARIES ${VILLASnode_LIBRARY})
# set(VILLASnode_INCLUDE_DIR "")
set(VILLASnode_INCLUDE_DIR "${villasnode-module_SOURCE_DIR}/include" "${villasnode-module_SOURCE_DIR}/common/include" "${villasnode-module_BINARY_DIR}/include" "${villasnode-module_BINARY_DIR}/common/include")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VILLASnode DEFAULT_MSG VILLASnode_LIBRARY)

set(VILLASnode_LIBRARIES ${VILLASnode_LIBRARY})
set(VILLASnode_INCLUDE_DIRS ${VILLASnode_INCLUDE_DIR})
# set(LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:${villasnode-module_SOURCE_DIR}/build/lib")
# message(STATUS "Path to villasnode library: " ${LD_LIBRARY_PATH})
