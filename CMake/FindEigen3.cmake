option(WITH_EIGEN_SUBMODULE "Build with Eigen as submodule" OFF)

if (WITH_EIGEN_SUBMODULE OR WIN32)
	# This could be used when an optimized version of eigen is required
	# add_subdirectory(Dependencies/eigen)
	find_path(EIGEN3_INCLUDE_DIR
		NAMES signature_of_eigen3_matrix_library
		PATHS
		  Dependencies/eigen
	)
else()
	set(TEMP_PATH ${CMAKE_MODULE_PATH})
	# We temporarily clear the module path to avoid recursion
	set(CMAKE_MODULE_PATH "")
	find_package(Eigen3 REQUIRED)
	set(CMAKE_MODULE_PATH ${TEMP_PATH})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIR)
