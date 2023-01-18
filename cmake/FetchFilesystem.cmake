include(FetchContent)
FetchContent_Declare(filesystem-module
	GIT_REPOSITORY https://github.com/gulrak/filesystem
	GIT_TAG v1.5.12
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(filesystem-module)

add_library(filesystem ALIAS ghc_filesystem)

add_compile_definitions(USE_GHC_FS)
