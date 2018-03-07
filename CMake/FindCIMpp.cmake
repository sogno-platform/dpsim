option(CIMPP_DIR "CIM++ installation directory" ../CIMpp)
option(WITH_CIM_SUBMODULE "Build with CIMpp as submodule" OFF)

set(CIM_VERSION 16v29a)	
set(USE_CIM_VERSION "IEC61970_16v29a")

if (WITH_CIM_SUBMODULE OR WIN32)
	add_subdirectory(Dependencies/libcimpp)	
	set(CIMPP_LIBRARY ${CMAKE_BINARY_DIR}/Dependencies/libcimpp/Debug/CIMParser.lib)
	set(CIMPP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Dependencies/libcimpp/src ${CMAKE_SOURCE_DIR}/Dependencies/libcimpp/16v29a)
	set(ARABICA_LIBRARY ${CMAKE_BINARY_DIR}/Dependencies/libcimpp/thirdparty/arabica/Debug/arabica.lib)
	set(ARABICA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Dependencies/libcimpp/thirdparty/arabica/include)
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
	#### Deprecated beginning ####
	#if (WIN32)
	if (NOT)
	find_path(ARABICA_INCLUDE_DIR
		NAMES convert
		PATH_SUFFIXES
			Arabica
		PATHS
			../Arabica/include
	)
	find_library(ARABICA_LIBRARY
		NAMES arabica
		PATH_SUFFIXES
			lib/static
			Debug
			Release
		PATHS
		../Arabica
	)
	endif ()
	#### Deprecated end ####
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CIMpp DEFAULT_MSG CIMPP_LIBRARY CIMPP_INCLUDE_DIR)
mark_as_advanced(CIMPP_INCLUDE_DIR CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY} ${ARABICA_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR} ${ARABICA_INCLUDE_DIR})
