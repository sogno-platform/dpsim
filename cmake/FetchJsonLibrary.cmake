include(FetchContent)
FetchContent_Declare(json-module
	GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
	GIT_TAG v3.9.1
)

FetchContent_MakeAvailable(json-module)
