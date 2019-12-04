set(CIMPP_DIR Dependencies/libcimpp)
option(WITH_CIM_SUBMODULE "Build with CIMpp as submodule" OFF)

set(CIM_VERSION "16v29a")	
set(USE_CIM_VERSION "IEC61970_16v29a")

include(FindPackageHandleStandardArgs)

if(WIN32 AND NOT WITH_CIM_SUBMODULE)

	set(CIMPP_INCLUDE_DIR 
		${CMAKE_CURRENT_SOURCE_DIR}/${CIMPP_DIR}/src 
		${CMAKE_CURRENT_SOURCE_DIR}/${CIMPP_DIR}/${CIM_VERSION}
	)

	find_library(CIMPP_LIBRARY
		NAMES libcimpp.lib
		PATH_SUFFIXES
			Debug
			Release
		PATHS
			${CMAKE_CURRENT_BINARY_DIR}/${CIMPP_DIR}	
	)

	set(ARABICA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${CIMPP_DIR}/thirdparty/arabica/include)

	find_library(ARABICA_LIBRARY
		NAMES arabica.lib
		PATH_SUFFIXES
			Debug
			Release
		PATHS
			${CMAKE_CURRENT_BINARY_DIR}/${CIMPP_DIR}/thirdparty/arabica	
	)

	find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
	mark_as_advanced(CIMPP_INCLUDE_DIR)

	if(CIMPP_LIBRARY AND ARABICA_LIBRARY)
		set(WITH_CIM_SUBMODULE OFF)
	else()
		set(WITH_CIM_SUBMODULE ON)
		set(CIMPP_INCLUDE_DIR "")
		set(CIMPP_LIBRARY "")
		set(ARABICA_INCLUDE_DIR "")
		set(ARABICA_LIBRARY "")
	endif()
endif()

if (WITH_CIM_SUBMODULE)
	# When adding CIMpp via a subdirectory, all the neccessary
	# details about include directories, library name and path
	# are associated with the CIMpp target 'libcimpp'
	# We therefore do not need to keep track of CIMPP_INCLUDE_DIR
	# seperately

	if(NOT WIN32)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
	endif()
	add_subdirectory(Dependencies/libcimpp)
	set(CIMPP_LIBRARY libcimpp)
	set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
	set(CIMPP_INCLUDE_DIR "")

	find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY)

elseif(NOT WIN32)
	find_path(CIMPP_INCLUDE_DIR
		NAMES CIMModel.hpp
		PATH_SUFFIXES
			cimpp/${CIM_VERSION}
			${CIM_VERSION}
			include/src
	)
	find_library(CIMPP_LIBRARY
		NAMES cimpp${CIM_VERSION}
		PATH_SUFFIXES
			lib/static
	)

	# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
	# if all listed variables are TRUE
	find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
	mark_as_advanced(CIMPP_INCLUDE_DIR)

endif()

mark_as_advanced(CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY} ${ARABICA_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR} ${ARABICA_INCLUDE_DIR})
