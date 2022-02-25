# Workaround for Eigen in Windows

include(FetchContent)
FetchContent_Declare(
  eigen-module
  GIT_REPOSITORY https://github.com/LennartSchu/eigen-klu
  GIT_TAG        main
)

FetchContent_GetProperties(eigen-module)
if(NOT eigen-module_POPULATED)
  FetchContent_Populate(eigen-module)
endif()

find_path(EIGEN3_INCLUDE_DIR
  NAMES signature_of_eigen3_matrix_library
	PATHS
	  ${eigen-module_SOURCE_DIR}
)

FetchContent_GetProperties(eigen-module)
message(STATUS "Path to eigen-module: " ${eigen-module_SOURCE_DIR})
