include(FetchContent)
FetchContent_Declare(pybind-module
	GIT_REPOSITORY https://github.com/pybind/pybind11.git
	GIT_TAG        v2.10.3
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(pybind-module)

set(pybind11_FOUND TRUE)
