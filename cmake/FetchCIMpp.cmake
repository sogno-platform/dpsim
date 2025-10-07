# cmake_minimum_required handled by your top-level CMakeLists
# Requires CMake >= 3.14 for FetchContent_MakeAvailable

set(CIM_VERSION "${CIM_VERSION}" CACHE STRING "CIM version (e.g. CGMES_2.4.15_16FEB2016 or 16v29a)")
if(NOT CIM_VERSION OR CIM_VERSION STREQUAL "")
	set(CIM_VERSION "CGMES_2.4.15_16FEB2016")
endif()

set(CIMPP_COMMIT "${CIMPP_COMMIT}" CACHE STRING "libcimpp git commit to fetch")
if(NOT DEFINED CIMPP_COMMIT OR CIMPP_COMMIT STREQUAL "")
	if(WIN32)
		# Windows builds currently rely on the older commit; update once validated there.
		set(CIMPP_COMMIT "1b11d5c17bedf0ae042628b42ecb4e49df70b2f6")
	else()
		# Linux and other platforms use the newer known-good commit.
		set(CIMPP_COMMIT "051ee4c311572fe92b30120b897d22deb253e162")
	endif()
endif()

message(STATUS "[CIMpp] CIM Version: ${CIM_VERSION}")
message(STATUS "[CIMpp] Commit: ${CIMPP_COMMIT}")

set(CGMES_BUILD OFF)

if(CIM_VERSION STREQUAL "16v29a")
	set(USE_CIM_VERSION "IEC61970_16v29a")
elseif(CIM_VERSION STREQUAL "CGMES_2.4.15_16FEB2016")
	set(USE_CIM_VERSION "CGMES_2.4.15_16FEB2016")
	set(CGMES_BUILD ON)
endif()

set(CIM_VERSION   "${CIM_VERSION}"   CACHE STRING "" FORCE)
set(CGMES_BUILD   "${CGMES_BUILD}"   CACHE BOOL   "" FORCE)
set(CIMPP_BUILD_DOC OFF              CACHE BOOL   "Build libcimpp docs (disabled)" FORCE)

include(FetchContent)

FetchContent_Declare(cimpp-module
	GIT_REPOSITORY https://github.com/sogno-platform/libcimpp.git
	GIT_TAG        ${CIMPP_COMMIT}
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(cimpp-module)

set(_candidates CIMPP::cimpp cimpp libcimpp)
set(_cimpp_target "")
foreach(tgt IN LISTS _candidates)
	if(TARGET ${tgt})
		set(_cimpp_target ${tgt})
		break()
	endif()
endforeach()

if(_cimpp_target STREQUAL "")
	message(FATAL_ERROR "[CIMpp] Fetched libcimpp but no library target was found (checked: ${_candidates}).")
endif()

if(NOT TARGET CIMPP::cimpp)
	if(_cimpp_target STREQUAL "CIMPP::cimpp")
	else()
		add_library(CIMPP::cimpp ALIAS ${_cimpp_target})
	endif()
endif()

set(CIMpp_FOUND TRUE CACHE BOOL "Fetched CIMpp successfully" FORCE)
set(CIMPP_CGMES_BUILD ${CGMES_BUILD} CACHE BOOL "Is this a CGMES build?" FORCE)
set(CIMPP_USE_VERSION "${USE_CIM_VERSION}" CACHE STRING "Resolved internal CIM version tag" FORCE)
