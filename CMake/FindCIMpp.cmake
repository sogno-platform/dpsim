option(CIMPP_DIR "CIM++ installation directory" ../CIMpp)
option(WITH_CIM_SUBMODULE "Build with CIMpp as submodule" OFF)

set(CIM_VERSION "16v29a")	
set(USE_CIM_VERSION "IEC61970_16v29a")

include(FindPackageHandleStandardArgs)

if (WITH_CIM_SUBMODULE OR WIN32)
	# When adding CIMpp via a subdirectory, all the neccessary
	# details about include directories, library name and path
	# are associated with the CIMpp target 'libcimpp'
	# We therefore do not need to keep track of CIMPP_INCLUDE_DIR
	# seperately

	add_subdirectory(Dependencies/libcimpp)
	set(CIMPP_LIBRARY libcimpp)

	find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY)
else()
	find_path(CIMPP_INCLUDE_DIR
		NAMES CIMModel.hpp
		PATH_SUFFIXES
			cimpp/${CIM_VERSION}
			${CIM_VERSION}
			include/src
		PATHS
			${CIMPP_DIR}
			../CIMpp
	)
	find_library(CIMPP_LIBRARY
		NAMES cimpp${CIM_VERSION}
		PATH_SUFFIXES
			lib/static
			Debug
			Release
		PATHS
			${CIMPP_DIR}
			../CIMpp		
	)

	# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
	# if all listed variables are TRUE
	find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
	mark_as_advanced(CIMPP_INCLUDE_DIR)
endif()

mark_as_advanced(CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY} ${ARABICA_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR} ${ARABICA_INCLUDE_DIR})