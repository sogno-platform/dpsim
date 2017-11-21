option(CIMPP_DIR "CIM++ installation directory" ../CIMpp)

if(WIN32)
	file(TO_CMAKE_PATH $ENV{APPDATA} APPDATA)
endif()

find_path(CIMPP_INCLUDE_DIR
	NAMES CIMModel.hpp
	PATH_SUFFIXES
		cimpp/16v29a_12v08
		cimpp/16v29a
		cimpp/17v07
	PATHS
		../CIMpp/include
		${APPDATA}/CIMpp/include
		${CIMPP_DIR}
)

find_library(CIMPP_LIBRARY
	NAMES cimpp${CIM_DIR}
	PATH_SUFFIXES
		lib/static
	PATHS
		../CIMpp
		${APPDATA}/CIMpp
		${CIMPP_DIR}
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
mark_as_advanced(CIMPP_INCLUDE_DIR CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR})
