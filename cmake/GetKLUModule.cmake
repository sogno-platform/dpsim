# Workaround for SuiteSparse in Windows

include(FetchContent)
FetchContent_Declare(
  klu-module
  GIT_REPOSITORY https://github.com/LennartSchu/SuiteSparse
  GIT_TAG        master
)

FetchContent_GetProperties(klu-module)
if(NOT klu-module_POPULATED)
  FetchContent_Populate(klu-module)
endif()

FetchContent_GetProperties(klu-module)

add_subdirectory(${klu-module_SOURCE_DIR} ${klu-module_BINARY_DIR})

set(SS_PATH ${CMAKE_SOURCE_DIR}/build)

set(KLU_LIBRARY ${SS_PATH}/libklu.a ${SS_PATH}/libbtf.a ${SS_PATH}/libamd.a ${SS_PATH}/libcolamd.a ${SS_PATH}/libsuitesparseconfig.a)

set(KLU_LIBRARIES ${KLU_LIBRARY})
set(KLU_INCLUDE_DIR ${klu-module_SOURCE_DIR}/KLU/Include ${klu-module_SOURCE_DIR}/BTF/Include ${klu-module_SOURCE_DIR}/COLAMD/Include ${klu-module_SOURCE_DIR}/AMD/Include ${klu-module_SOURCE_DIR}/SuiteSparse_config/)
set(KLU_INCLUDE_DIRS ${KLU_INCLUDE_DIR})
message(STATUS "Path to klu-module: " ${klu-module_SOURCE_DIR})
