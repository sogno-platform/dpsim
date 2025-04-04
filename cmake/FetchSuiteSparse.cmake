include(FetchContent)

FetchContent_Declare(
	suitesparse-module
	URL https://github.com/dpsim-simulator/SuiteSparse/releases/download/release-v5.10.6/SuiteSparse-release-v5.10.6.tar.gz
)

FetchContent_MakeAvailable(suitesparse-module)

