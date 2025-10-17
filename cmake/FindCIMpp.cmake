cmake_minimum_required(VERSION 3.12)

set(CIM_VERSION "${CIM_VERSION}" CACHE STRING "CIM version (e.g. CGMES_2.4.15_16FEB2016)")
set(CIMPP_ROOT  "${CIMPP_ROOT}"  CACHE PATH   "CIMpp install prefix (/usr, /usr/local, /opt/cimpp)")

if(NOT CIM_VERSION OR CIM_VERSION STREQUAL "")
	set(CIM_VERSION "CGMES_2.4.15_16FEB2016")
	message(STATUS "[CIMpp] CIM_VERSION not provided, defaulting to ${CIM_VERSION}")
else()
	message(STATUS "[CIMpp] CIM_VERSION provided: ${CIM_VERSION}")
endif()

if(NOT CIMPP_ROOT OR CIMPP_ROOT STREQUAL "")
	set(CIMPP_ROOT "/usr/local")
	message(STATUS "[CIMpp] CIMPP_ROOT not provided, defaulting to ${CIMPP_ROOT}")
else()
	message(STATUS "[CIMpp] CIMPP_ROOT provided: ${CIMPP_ROOT}")
endif()

# Detect CGMES builds and expose a flag
set(CGMES_BUILD OFF)
if(CIM_VERSION MATCHES "^CGMES_")
	set(CGMES_BUILD ON)
endif()

# Hints
set(_hints ${CIMPP_ROOT} $ENV{CIMPP_ROOT} /usr /usr/local)
list(FILTER _hints EXCLUDE REGEX "^$")

# version: .../include/cimpp/<version>
find_path(CIMPP_VER_INCLUDE_DIR
	NAMES Line.hpp
	HINTS ${_hints}
	PATH_SUFFIXES
		include/cimpp/${CIM_VERSION}
		include/cimpp)

# src: .../include/cimpp/src
find_path(CIMPP_SRC_INCLUDE_DIR
	NAMES CIMFile.hpp
	HINTS ${_hints}
	PATH_SUFFIXES
		include/cimpp
		include/cimpp/${CIM_VERSION}
		include/cimpp/src
		include/cimpp/${CIM_VERSION}/src)

# static: .../include/cimpp/static
find_path(CIMPP_STATIC_INCLUDE_DIR
	NAMES BaseClass.hpp
	HINTS ${_hints}
	PATH_SUFFIXES
		include/cimpp/${CIM_VERSION}
		include/cimpp/${CIM_VERSION}/static
		include/cimpp/static
		include/cimpp)

# static/IEC61970: .../include/cimpp/static/IEC61970
find_path(CIMPP_STATIC_IEC61970_INCLUDE_DIR
	NAMES IEC61970CIMVersion.h
	HINTS ${_hints}
	PATH_SUFFIXES
		include/cimpp/${CIM_VERSION}/IEC61970
		include/cimpp/${CIM_VERSION}/static/IEC61970
		include/cimpp/static/IEC61970
		include/cimpp/IEC61970)

# ---- Library (search specific directories only)
set(_cimpp_library_dirs "")
foreach(_root IN LISTS _hints)
	if(_root)
		list(APPEND _cimpp_library_dirs
			"${_root}/lib"
			"${_root}/lib64"
			"${_root}/lib/x86_64-linux-gnu")
	endif()
endforeach()
list(APPEND _cimpp_library_dirs "/lib" "/lib64" "/lib/x86_64-linux-gnu")
list(REMOVE_DUPLICATES _cimpp_library_dirs)

find_library(CIMPP_LIBRARY
	NAMES cimpp${CIM_VERSION} cimppCGMES_2.4.15_16FEB2016 cimpp
	PATHS ${_cimpp_library_dirs}
	NO_DEFAULT_PATH)

# ---- Revalidate discovered paths (avoid cached empty or stale results)
macro(_cimpp_recheck _var)
	if(${_var} AND NOT EXISTS "${${_var}}")
		unset(${_var} CACHE)
		set(${_var} "")
	endif()
endmacro()

_cimpp_recheck(CIMPP_SRC_INCLUDE_DIR)
_cimpp_recheck(CIMPP_VER_INCLUDE_DIR)
_cimpp_recheck(CIMPP_STATIC_INCLUDE_DIR)
_cimpp_recheck(CIMPP_STATIC_IEC61970_INCLUDE_DIR)
_cimpp_recheck(CIMPP_LIBRARY)

# ---- Validate
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CIMpp
	REQUIRED_VARS
	CIMPP_LIBRARY
	CIMPP_SRC_INCLUDE_DIR
	CIMPP_VER_INCLUDE_DIR
	CIMPP_STATIC_INCLUDE_DIR
	CIMPP_STATIC_IEC61970_INCLUDE_DIR
	FAIL_MESSAGE "Set CIMPP_ROOT and CIM_VERSION so that the include paths are found.")

if(NOT TARGET CIMPP::cimpp)
	add_library(CIMPP::cimpp UNKNOWN IMPORTED)
	set(_incs
		"${CIMPP_SRC_INCLUDE_DIR}"
		"${CIMPP_VER_INCLUDE_DIR}"
		"${CIMPP_STATIC_INCLUDE_DIR}"
		"${CIMPP_STATIC_IEC61970_INCLUDE_DIR}")
	foreach(_inc IN LISTS _incs)
		if(_inc STREQUAL "" OR NOT EXISTS "${_inc}")
			list(REMOVE_ITEM _incs "${_inc}")
		endif()
	endforeach()
	list(REMOVE_DUPLICATES _incs)
	if(NOT CIMPP_LIBRARY OR _incs STREQUAL "")
		message(WARNING "[CIMpp] Skipping CIMPP::cimpp target because include paths or library are missing")
		unset(CIMPP_LIBRARY CACHE)
		set(CIMPP_LIBRARY "")
	else()
		set_target_properties(CIMPP::cimpp PROPERTIES
		IMPORTED_LOCATION "${CIMPP_LIBRARY}"
		INTERFACE_INCLUDE_DIRECTORIES "${_incs}")
	endif()
endif()

message(STATUS "[CIMpp] src     = ${CIMPP_SRC_INCLUDE_DIR}")
message(STATUS "[CIMpp] version  = ${CIMPP_VER_INCLUDE_DIR}")
message(STATUS "[CIMpp] static   = ${CIMPP_STATIC_INCLUDE_DIR}")
message(STATUS "[CIMpp] IEC61970 = ${CIMPP_STATIC_IEC61970_INCLUDE_DIR}")
message(STATUS "[CIMpp] lib      = ${CIMPP_LIBRARY}")
