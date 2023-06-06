include(FetchContent)

FetchContent_Declare(
	nicslu-module
	GIT_REPOSITORY git@git-ce.rwth-aachen.de:acs/private/research/rte/nicslu-customlu.git
	GIT_TAG        main
)

FetchContent_GetProperties(nicslu-module)
if(NOT nicslu-module_POPULATED)
	FetchContent_Populate(nicslu-module)
endif()

FetchContent_MakeAvailable(nicslu-module)

add_subdirectory(${nicslu-module_SOURCE_DIR} ${nicslu-module_BINARY_DIR})

set(NICSLU_LIBRARY ${nicslu-module_BINARY_DIR}/libnicslu.a)

set(NICSLU_LIBRARIES ${NICSLU_LIBRARY})
set(NICSLU_INCLUDE_DIR ${nicslu-module_SOURCE_DIR}/include)
set(NICSLU_INCLUDE_DIRS ${NICSLU_INCLUDE_DIR})

message(STATUS "Path to nicslu-module: " ${nicslu-module_SOURCE_DIR})

