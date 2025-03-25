option(SPDLOG_BUILD_TESTING "Build spdlog tests" OFF)
option(SPDLOG_BUILD_BENCH "Build spdlog benchmarks" OFF)
option(SPDLOG_BUILD_EXAMPLES "Build spdlog examples" OFF)

include(FetchContent)
FetchContent_Declare(spdlog-module
	GIT_REPOSITORY https://github.com/gabime/spdlog.git
	GIT_TAG        v1.15.0
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(spdlog-module)
