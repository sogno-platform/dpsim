option(CIMPP_DIR "CIM++ installation directory" ../CIMpp)

if(NOT DEFINED CIM_VERSION)
	set(CIM_VERSION 16v29a)
endif()


find_path(CIMPP_INCLUDE_DIR
	NAMES CIMModel.hpp
	PATH_SUFFIXES
		cimpp/${CIM_VERSION}
		${CIM_VERSION}
		include/src
	PATHS
		${CIMPP_DIR}
		../CIMpp
		../Libraries/CIMpp
		../dpsim-libraries/CIMpp
)

find_library(CIMPP_LIBRARY
	NAMES cimpp${CIM_VERSION}
	PATH_SUFFIXES
		lib/static
		Debug
		Release
	PATHS
		../CIMpp
		../Libraries/CIMpp
		../dpsim-libraries/CIMpp
		${CIMPP_DIR}
)

if (WIN32)
	find_path(ARABICA_INCLUDE_DIR
		NAMES convert
		PATH_SUFFIXES
			Arabica
		PATHS
			../Arabica/include
			../Libraries/Arabica/include
			../dpsim-libraries/Arabica/include
	)

	find_library(ARABICA_LIBRARY
		NAMES arabica
		PATH_SUFFIXES
			lib/static
			Debug
			Release
		PATHS
		../Arabica
		../Libraries/Arabica
		../dpsim-libraries/Arabica
	)
endif ()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
mark_as_advanced(CIMPP_INCLUDE_DIR CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY} ${ARABICA_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR} ${ARABICA_INCLUDE_DIR})
