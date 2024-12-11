if(NOT CIM_VERSION)
	set(CIM_VERSION "CGMES_2.4.15_16FEB2016")
endif()

# Allow overriding the commit hash externally
if(NOT DEFINED CIMPP_COMMIT)
    set(CIMPP_COMMIT "1b11d5c17bedf0ae042628b42ecb4e49df70b2f6") # Default commit
endif()

message(STATUS "CIM Version: ${CIM_VERSION}")

if(CIM_VERSION STREQUAL "16v29a")
	set(USE_CIM_VERSION "IEC61970_16v29a")
elseif(CIM_VERSION STREQUAL "CGMES_2.4.15_16FEB2016")
	set(USE_CIM_VERSION "CGMES_2.4.15_16FEB2016")
	set(CGMES_BUILD ON)
endif()

set(CIMPP_BUILD_DOC OFF)

include(FetchContent)
FetchContent_Declare(cimpp-module
	GIT_REPOSITORY https://github.com/sogno-platform/libcimpp.git
	GIT_PROGRESS   TRUE
	GIT_TAG        ${CIMPP_COMMIT}
)

FetchContent_MakeAvailable(cimpp-module)

set(CIMpp_FOUND TRUE)
