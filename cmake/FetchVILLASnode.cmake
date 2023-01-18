include(FetchContent)
FetchContent_Declare(villas-node-module
	GIT_REPOSITORY https://github.com/VILLASframework/node.git
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(villas-node-module)

set(VILLASnode_FOUND TRUE)
