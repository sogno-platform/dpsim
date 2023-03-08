include(FetchContent)

FetchContent_Declare(
	klu-module
	URL https://github.com/dpsim-simulator/SuiteSparse/releases/download/release-v5.10.6/SuiteSparse-release-v5.10.6.tar.gz
)

FetchContent_GetProperties(klu-module)
if(NOT klu-module_POPULATED)
  FetchContent_Populate(klu-module)
endif()

FetchContent_GetProperties(klu-module)

add_subdirectory(${klu-module_SOURCE_DIR} ${klu-module_BINARY_DIR})

set(SS_PATH ${CMAKE_SOURCE_DIR}/build)

if (CMAKE_BUILD_TYPE MATCHES Debug)
  set(KLU_LIBRARY ${SS_PATH}/libklu_debug.a ${SS_PATH}/libbtf_debug.a ${SS_PATH}/libamd_debug.a ${SS_PATH}/libcolamd_debug.a ${SS_PATH}/libsuitesparseconfig_debug.a)
else()
  set(KLU_LIBRARY ${SS_PATH}/libklu.a ${SS_PATH}/libbtf.a ${SS_PATH}/libamd.a ${SS_PATH}/libcolamd.a ${SS_PATH}/libsuitesparseconfig.a)
endif(CMAKE_BUILD_TYPE MATCHES Debug)

set(KLU_LIBRARIES ${KLU_LIBRARY})
set(KLU_INCLUDE_DIR ${klu-module_SOURCE_DIR}/KLU/Include ${klu-module_SOURCE_DIR}/BTF/Include ${klu-module_SOURCE_DIR}/COLAMD/Include ${klu-module_SOURCE_DIR}/AMD/Include ${klu-module_SOURCE_DIR}/SuiteSparse_config/)
set(KLU_INCLUDE_DIRS ${KLU_INCLUDE_DIR})
message(STATUS "Path to klu-module: " ${klu-module_SOURCE_DIR})
