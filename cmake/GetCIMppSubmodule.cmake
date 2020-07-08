# Workaround for cimpp on Windows and as submodule

set(CIM_VERSION "16v29a")
set(USE_CIM_VERSION "IEC61970_16v29a")

include(FetchContent)
FetchContent_Declare(
  cimpp-submodule
  GIT_REPOSITORY https://github.com/cim-iec/libcimpp.git
  GIT_TAG        master
)

FetchContent_GetProperties(cimpp-submodule)
if(NOT cimpp-submodule_POPULATED)
  FetchContent_Populate(cimpp-submodule)
endif()

FetchContent_GetProperties(cimpp-submodule)
message(STATUS "Path to cimpp-submodule: " ${cimpp-submodule_SOURCE_DIR})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# When adding CIMpp via a subdirectory, all the neccessary
# details about include directories, library name and path
# are associated with the CIMpp target 'libcimpp'
# We do not need to keep track of CIMPP_INCLUDE_DIR.
add_subdirectory(${cimpp-submodule_SOURCE_DIR} ${cimpp-submodule_BINARY_DIR})
set(CIMPP_LIBRARY libcimpp)
set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIR "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR})