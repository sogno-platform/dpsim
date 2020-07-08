# Workaround for Eigen in Windows

include(FetchContent)
FetchContent_Declare(
  eigen-submodule
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.3.7
)

FetchContent_GetProperties(eigen-submodule)
if(NOT eigen-submodule_POPULATED)
  FetchContent_Populate(eigen-submodule)
endif()

find_path(EIGEN3_INCLUDE_DIR
  NAMES signature_of_eigen3_matrix_library
	PATHS
	  ${eigen-submodule_SOURCE_DIR}
)

FetchContent_GetProperties(eigen-submodule)
message(STATUS "Path to eigen-submodule: " ${eigen-submodule_SOURCE_DIR})
