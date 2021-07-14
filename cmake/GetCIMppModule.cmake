# Workaround for cimpp on Windows and as module

if(NOT CIM_VERSION)
	set(CIM_VERSION "CGMES_2.4.15_16FEB2016")
endif()

message(STATUS "CIM Version: ${CIM_VERSION}")

if(CIM_VERSION STREQUAL "16v29a")
	set(USE_CIM_VERSION "IEC61970_16v29a")
endif()
if(CIM_VERSION STREQUAL "CGMES_2.4.15_16FEB2016")
	set(USE_CIM_VERSION "CGMES_2.4.15_16FEB2016")
  set(CGMES_BUILD ON)
endif()

include(FetchContent)
FetchContent_Declare(
  cimpp-module
  GIT_REPOSITORY https://github.com/cim-iec/libcimpp.git
  GIT_TAG        master
)

FetchContent_GetProperties(cimpp-module)
if(NOT cimpp-module_POPULATED)
  FetchContent_Populate(cimpp-module)
endif()

FetchContent_GetProperties(cimpp-module)
message(STATUS "Path to cimpp-module: " ${cimpp-module_SOURCE_DIR})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# When adding CIMpp via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the CIMpp target 'libcimpp'
# We do not need to keep track of CIMPP_INCLUDE_DIR.
add_subdirectory(${cimpp-module_SOURCE_DIR} ${cimpp-module_BINARY_DIR})
set(CIMPP_LIBRARY libcimpp)
set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIR "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR})