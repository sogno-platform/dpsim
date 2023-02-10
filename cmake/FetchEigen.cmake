include(FetchContent)
FetchContent_Declare(eigen-module
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG        3fe8c511042ab1af84c2f91015463708f969255e
	GIT_PROGRESS   TRUE
)

set(EIGEN_BUILD_DOC OFF)

FetchContent_MakeAvailable(eigen-module)
