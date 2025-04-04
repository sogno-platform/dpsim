include(FetchContent)
FetchContent_Declare(readerwriterqueue-module
	GIT_REPOSITORY https://github.com/cameron314/readerwriterqueue
	GIT_TAG        v1.0.6
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(readerwriterqueue-module)

add_library(readerwriterqueue::readerwriterqueue ALIAS readerwriterqueue)
