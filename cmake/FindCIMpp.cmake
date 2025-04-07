if(NOT CIM_VERSION)
	set(CIM_VERSION "CGMES_2.4.15_16FEB2016")
endif()

message(STATUS "CIM Version: ${CIM_VERSION}")

if(CIM_VERSION STREQUAL "16v29a")
	set(USE_CIM_VERSION "IEC61970_16v29a")
elseif(CIM_VERSION STREQUAL "CGMES_2.4.15_16FEB2016")
	set(USE_CIM_VERSION "CGMES_2.4.15_16FEB2016")
	set(CGMES_BUILD ON)
endif()

find_path(CIMPP_INCLUDE_DIR
	NAMES CIMModel.hpp
	PATH_SUFFIXES
		include/cimpp/${CIM_VERSION}
		include/cimpp/${USE_CIM_VERSION}
)

find_library(CIMPP_LIBRARY
	NAMES
		cimpp${CIM_VERSION}
		cimpp${USE_CIM_VERSION}
	PATH_SUFFIXES
		lib/static
		lib/cimpp
)

set(CIMPP_LIBRARIES
	${CIMPP_LIBRARY}
	${ARABICA_LIBRARY}
)

set(CIMPP_INCLUDE_DIRS
	${CIMPP_INCLUDE_DIR}
	${ARABICA_INCLUDE_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)

mark_as_advanced(CIMPP_LIBRARY CIMPP_INCLUDE_DIR)

add_library(libcimpp INTERFACE)
target_link_libraries(libcimpp INTERFACE ${CIMPP_LIBRARIES})
target_include_directories(libcimpp INTERFACE ${CIMPP_INCLUDE_DIRS})
