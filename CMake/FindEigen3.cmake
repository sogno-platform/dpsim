# We first try to find a version inside our local Libraries folder
find_path(EIGEN3_INCLUDE_DIR
	NAMES signature_of_eigen3_matrix_library
	PATHS
		../Eigen
		../Libraries/Eigen
		../dpsim-libraries/Eigen
)

if(NOT EIGEN3_INCLUDE_DIR)
	set(TEMP_PATH ${CMAKE_MODULE_PATH})

	# We temporarily clear the module path to avoid recursion
	set(CMAKE_MODULE_PATH "")
	find_package(Eigen3 REQUIRED)
	set(CMAKE_MODULE_PATH ${TEMP_PATH})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIR)
