include(FetchContent)
FetchContent_Declare(eigen-module
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG        3.3.7
)

FetchContent_MakeAvailable(eigen-module)

# Add missing alias for Eigen 3.3
# See: https://gitlab.com/libeigen/eigen/-/issues/2440
# TODO: Remove for Eigen >= 3.4
add_library(Eigen3::Eigen ALIAS eigen)
