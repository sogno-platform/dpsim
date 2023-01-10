include(FetchContent)
FetchContent_Declare(villas-node-module
	GIT_REPOSITORY https://github.com/VILLASframework/node.git
)

FetchContent_MakeAvailable(villas-node-module)

set(VILLASnode_FOUND TRUE)
